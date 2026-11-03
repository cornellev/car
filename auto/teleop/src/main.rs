//! Drive the car from ackermann commands on `cev/commands/control`.
//! Commands are radians and m/s. The Pico wants a steering position and a duty, both in [-1, 1].
//! Telemetry comes back as wheel rot/s and an encoder count, published as rot/s and degrees.
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

// shortcut: 900 firmware steps span the measured 13.5 deg stop. Replace both once counted.
const STEERING_LIMIT_DEG: f32 = 13.5;
const HALF_RANGE_STEPS: f32 = 900.0;
const PULSES_PER_DEG: f32 = HALF_RANGE_STEPS / STEERING_LIMIT_DEG;
// Full-scale duty is about 2 m/s. Feedforward only; a speed loop comes later.
const MAX_SPEED_MPS: f32 = 2.0;
// Encoder center and counts per revolution. Wrap is 0 until the magnetic encoder is wired,
// so only COUNTS_PER_DEG is used. Positive counts are clockwise (right).
const CENTER_COUNTS: f32 = 0.0;
const ENCODER_COUNTS_PER_REV: f32 = 0.0;
const COUNTS_PER_DEG: f32 = PULSES_PER_DEG;

fn steering_command(angle_rad: f32) -> f32 {
    let degrees = angle_rad.to_degrees();
    // A positive Pico position turns clockwise (right). REP-103 positive is left.
    (-degrees * PULSES_PER_DEG / HALF_RANGE_STEPS).clamp(-1.0, 1.0)
}

fn drivetrain_command(speed_mps: f32) -> f32 {
    (speed_mps / MAX_SPEED_MPS).clamp(-1.0, 1.0)
}

fn steering_degrees(raw: i32, wrap: i32) -> f32 {
    let counts = wrap as f32 * ENCODER_COUNTS_PER_REV + raw as f32 - CENTER_COUNTS;
    -(counts / COUNTS_PER_DEG)
}

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
// and returns the newest valid telemetry (left rot/s, right rot/s, encoder raw, encoder wrap)
fn read_telemetry(
    port: &mut dyn SerialPort,
    rx: &mut Vec<u8>,
) -> std::io::Result<Option<(f32, f32, i32, i32)>> {
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

                    let steering = steering_command(ackermann.steering_angle as f32);
                    let drivetrain = drivetrain_command(ackermann.speed as f32);

                    if let Some(p) = port.as_mut()
                        && let Err(e) = p.write_all(&protocol::encode_drive(steering, drivetrain))
                    {
                        eprintln!("write failed, reopening: {e}");
                        port = None;
                    }
                    if let Some(p) = port.as_mut() {
                        match read_telemetry(p.as_mut(), &mut rx) {
                            // one line, rewritten in place
                            Ok(Some((left, right, raw, wrap))) => {
                                let heading = steering_degrees(raw, wrap);
                                if let Err(e) = steering_heading_pub
                                    .put(heading.to_string())
                                    .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                                    .await
                                {
                                    eprintln!("\nsteering heading publish failed: {e}");
                                }
                                eprint!("\rrps  left {left:7.2}  right {right:7.2}  steering raw {raw:7} wrap {wrap:3} ({heading:6.1} deg)  ");
                                for (publisher, rps) in [(&right_wheel_pub, right), (&left_wheel_pub, left)] {
                                    if let Err(e) = publisher
                                        .put(rps.to_string())
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steering_command_is_left_positive_and_clamped() {
        assert!(steering_command(0.0).abs() < 1e-6);
        let full_left = steering_command(STEERING_LIMIT_DEG.to_radians());
        assert!((full_left - (-1.0)).abs() < 1e-5, "{full_left}");
        assert_eq!(steering_command(40.0_f32.to_radians()), -1.0);
        assert_eq!(steering_command(-40.0_f32.to_radians()), 1.0);
    }

    #[test]
    fn drivetrain_command_is_duty_at_two_meters_per_second() {
        assert!((drivetrain_command(0.8) - 0.4).abs() < 1e-6);
        assert_eq!(drivetrain_command(5.0), 1.0);
        assert_eq!(drivetrain_command(-5.0), -1.0);
    }

    #[test]
    fn steering_degrees_negates_clockwise_counts() {
        let steps = HALF_RANGE_STEPS as i32;
        let degrees = steering_degrees(steps, 0);
        assert!((degrees - (-STEERING_LIMIT_DEG)).abs() < 1e-4, "{degrees}");
        assert!(steering_degrees(-steps, 0) > 0.0);
    }
}
