//! Drive the car from ackermann commands on `cev/commands/control`.
//! Shows serial telemetry and publishes wheel rotations/s and steering heading in degrees.
//!
//! usage: teleop [--dry-run] [serial port]
//! --dry-run prints the commands instead of sending them.

mod protocol;
mod config;
mod ackermann;

use std::io::Write;
use std::time::{Duration, Instant};

use serialport::SerialPort;

const DEFAULT_PORT: &str = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_505374175A8A7018-if00";
const SEND_PERIOD: Duration = Duration::from_millis(20); // 50 Hz, well inside the firmware's 200 ms timeout
const REOPEN_PERIOD: Duration = Duration::from_secs(1);

// Topic names retain the rpm suffix; payloads are JSON floats in rotations/second.
const RIGHT_WHEEL_KEY: &str = "cev/sensors/wheels/right_rpm";
const LEFT_WHEEL_KEY: &str = "cev/sensors/wheels/left_rpm";
const STEERING_ANGLE_KEY: &str = "cev/sensors/steering/steering_angle";

// TODO: calibrate the steering linkage. Placeholder scale: +/-900 pulses -> +/-45 degrees,
// boot center = 0. On merceves a positive Pico command and positive pulses both turn
// clockwise (right), so both are negated to keep REP-103 positive = left.
const STEERING_DEGREES_PER_PULSE: f32 = -45.0 / 900.0;

const MAX_FRAME_LEN: usize = 32; // longer runs without a 0x00 are garbage

fn open_port(path: &str) -> Option<Box<dyn SerialPort>> {
    // the pico only reads usb input while dtr is set; linux sets it on open, windows doesn't
    match serialport::new(path, 115_200)
        .timeout(Duration::from_millis(100))
        .dtr_on_open(true)
        .open()
    {
        Ok(port) => {
            eprintln!("opened {path}");
            Some(port)
        }
        Err(e) => {
            eprintln!("can't open {path}: {e}");
            None
        }
    }
}

// reads whatever has arrived without blocking, splits it into frames on 0x00,
// and returns the newest valid telemetry (left rpm, right rpm, steering pulses)
fn read_telemetry(
    port: &mut dyn SerialPort,
    rx: &mut Vec<u8>,
) -> std::io::Result<Option<(f32, f32, i32)>> {
    let mut incoming = vec![0; port.bytes_to_read()? as usize];
    port.read_exact(&mut incoming)?;

    let mut latest = None;
    for byte in incoming {
        if byte == 0x00 {
            if let Some(telemetry) = protocol::decode_telemetry(rx) {
                latest = Some(telemetry);
            }
            rx.clear();
        } else if rx.len() < MAX_FRAME_LEN {
            rx.push(byte);
        } else {
            rx.clear(); // the rest of this frame then fails to decode and gets dropped
        }
    }
    Ok(latest)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut dry_run = config::DRY_RUN;
    let mut port_path = DEFAULT_PORT.to_string();
    for arg in std::env::args().skip(1) {
        if arg == "--dry-run" {
            dry_run = true;
        } else {
            port_path = arg;
        }
    }

    let mut port: Option<Box<dyn SerialPort>> = None;
    let mut last_open_attempt: Option<Instant> = None;
    let mut rx = Vec::new();

    let mut ackermann = ackermann::AckermannCommand::default();

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let ack_sub = session.declare_subscriber(config::ACKERMANN_KEY).await?;
    let right_wheel_pub = session.declare_publisher(RIGHT_WHEEL_KEY).await?;
    let left_wheel_pub = session.declare_publisher(LEFT_WHEEL_KEY).await?;
    let steering_heading_pub = session.declare_publisher(STEERING_ANGLE_KEY).await?;

    loop {
        tokio::select! {
            result = ack_sub.recv_async() => {
                let Ok(sample) = result else {
                    eprintln!("ackermann subscription closed");
                    break;
                };

                let payload = sample.payload().to_bytes();

                match serde_json::from_slice::<ackermann::AckermannCommand>(&payload) {
                    Ok(command) => {
                        ackermann.update(command.steering_angle, command.speed);
                    }
                    Err(e) => {
                        eprintln!("failed to deserialize ackermann command: {e}");
                    }
                }
            }

            // a loop that just reads the ackermann command and sends it to the serial port at a fixed rate
            _ = tokio::time::sleep(SEND_PERIOD) => {
                if dry_run {
                    println!("steering: {:.3}, drivetrain: {:.3}", ackermann.steering_angle, ackermann.speed);
                    continue;
                } else {
                    if port.is_none() && last_open_attempt.is_none_or(|t| t.elapsed() >= REOPEN_PERIOD) {
                        last_open_attempt = Some(Instant::now());
                        port = open_port(&port_path);
                        rx.clear();
                    }

                    let steering = -(ackermann.steering_angle as f32);
                    let drivetrain = ackermann.speed as f32;

                    if let Some(p) = port.as_mut()
                        && let Err(e) = p.write_all(&protocol::encode_drive(steering, drivetrain))
                    {
                        eprintln!("write failed, reopening: {e}");
                        port = None;
                    }
                    if let Some(p) = port.as_mut() {
                        match read_telemetry(p.as_mut(), &mut rx) {
                            // one line, rewritten in place
                            Ok(Some((left, right, position))) => {
                                let heading = position as f32 * STEERING_DEGREES_PER_PULSE;
                                if let Err(e) = steering_heading_pub
                                    .put(heading.to_string())
                                    .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                                    .await
                                {
                                    eprintln!("\nsteering heading publish failed: {e}");
                                }
                                eprint!("\rrpm  left {left:7.1}  right {right:7.1}  steering {position:7} pulses ({heading:6.1} deg)  ");
                                for (publisher, rpm) in [(&right_wheel_pub, right), (&left_wheel_pub, left)] {
                                    if let Err(e) = publisher
                                        .put((rpm / 60.0).to_string())
                                        .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                                        .await
                                    {
                                        eprintln!("\nwheel telemetry publish failed: {e}");
                                    }
                                }
                            }
                            Ok(None) => {}
                            Err(e) => {
                                eprintln!("\nread failed, reopening: {e}");
                                port = None;
                            }
                        }
                    }
                }
            }
        }
    }
    Ok(())
}
