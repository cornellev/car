//! Drive the car from ackermann commands on `cev/commands/control`.
//! Commands arrive in radians and m/s. Steering is closed-loop on the Pico's
//! encoder angle and clamped at 11 degrees. Drivetrain duty is full scale at 2 m/s.
//! Telemetry publishes wheel rot/s and that encoder angle in degrees.
//!
//! usage: teleop [--dry-run] [serial port]
//! --dry-run prints the commands instead of sending them.

mod ackermann;
mod config;
mod protocol;

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

const MAX_FRAME_LEN: usize = 32; // longer runs without a 0x00 are garbage

const MAX_STEERING_DEG: f32 = 11.0;
// shortcut: untuned, set wheels-up in WP6. Command change per degree of error, once per telemetry frame.
const STEERING_GAIN: f32 = 0.01;
const STEERING_DEADBAND_DEG: f32 = 0.05;
// shortcut: unmeasured; full duty is about 2 m/s. Feedforward only.
const MAX_SPEED_MPS: f32 = 2.0;

fn steering_target_deg(angle_rad: f32) -> f32 {
    angle_rad.to_degrees().clamp(-MAX_STEERING_DEG, MAX_STEERING_DEG)
}

// Positive angle is left. A positive Pico command steers right, so the correction is negated.
fn steering_step(command: f32, target_deg: f32, measured_deg: f32) -> f32 {
    let error = target_deg - measured_deg;
    if error.abs() <= STEERING_DEADBAND_DEG {
        command
    } else {
        (command - STEERING_GAIN * error).clamp(-1.0, 1.0)
    }
}

fn drivetrain_command(speed_mps: f32) -> f32 {
    (speed_mps / MAX_SPEED_MPS).clamp(-1.0, 1.0)
}

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
// and returns the newest valid telemetry, including the raw encoder sample
fn read_telemetry(
    port: &mut dyn SerialPort,
    rx: &mut Vec<u8>,
) -> std::io::Result<Option<protocol::Telemetry>> {
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
    let mut steering_cmd = 0.0f32;

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
                    if port.is_none() {
                        steering_cmd = 0.0;
                        if last_open_attempt.is_none_or(|t| t.elapsed() >= REOPEN_PERIOD) {
                            last_open_attempt = Some(Instant::now());
                            port = open_port(&port_path);
                            rx.clear();
                        }
                    }

                    let target_deg = steering_target_deg(ackermann.steering_angle as f32);
                    let drivetrain = drivetrain_command(ackermann.speed as f32);

                    if let Some(p) = port.as_mut()
                        && let Err(e) = p.write_all(&protocol::encode_drive(steering_cmd, drivetrain))
                    {
                        eprintln!("write failed, reopening: {e}");
                        port = None;
                        steering_cmd = 0.0;
                    }
                    if let Some(p) = port.as_mut() {
                        match read_telemetry(p.as_mut(), &mut rx) {
                            // one line, rewritten in place
                            Ok(Some(telemetry)) => {
                                let left = telemetry.left_rps;
                                let right = telemetry.right_rps;
                                let position = telemetry.steering_pulses;
                                let encoder = match telemetry.encoder {
                                    Some(sample) => format!("raw={} status=0x{:02X}", sample.raw, sample.status),
                                    None => "I2C failed".to_string(),
                                };
                                let heading = telemetry.steering_degrees.unwrap_or(f32::NAN);
                                steering_cmd = match telemetry.steering_degrees {
                                    Some(measured) => steering_step(steering_cmd, target_deg, measured),
                                    None => 0.0,
                                };
                                if let Some(heading) = telemetry.steering_degrees {
                                    if let Err(e) = steering_heading_pub
                                        .put(heading.to_string())
                                        .encoding(zenoh::bytes::Encoding::APPLICATION_JSON)
                                        .await
                                    {
                                        eprintln!("\nsteering heading publish failed: {e}");
                                    }
                                }
                                eprint!("\rrps  left {left:7.10}  right {right:7.10}  steering {position:7} pulses ({heading:6.1} deg)  target {target_deg:5.2} deg cmd {steering_cmd:+.3}  encoder {encoder:24}  ");
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
                                steering_cmd = 0.0;
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
    fn steering_target_clamps_at_eleven_degrees() {
        assert!((steering_target_deg(5.0_f32.to_radians()) - 5.0).abs() < 1e-4);
        assert!((steering_target_deg(40.0_f32.to_radians()) - 11.0).abs() < 1e-4);
        assert!((steering_target_deg((-40.0_f32).to_radians()) - (-11.0)).abs() < 1e-4);
    }

    #[test]
    fn steering_step_turns_left_when_measured_is_right_of_target() {
        let command = steering_step(0.0, 0.0, -2.0);
        assert!(command < 0.0, "{command}");
        let command = steering_step(0.0, 0.0, 2.0);
        assert!(command > 0.0, "{command}");
    }

    #[test]
    fn steering_step_holds_inside_the_deadband_and_clamps() {
        assert_eq!(steering_step(0.4, 1.0, 1.0 + STEERING_DEADBAND_DEG / 2.0), 0.4);
        assert_eq!(steering_step(0.0, 200.0, 0.0), -1.0);
        assert_eq!(steering_step(0.0, -200.0, 0.0), 1.0);
    }

    #[test]
    fn steering_step_converges_for_a_linear_plant() {
        for gain in [15.0_f32, 30.0] {
            let target = 5.0;
            let mut command = 0.0;
            let mut measured = 0.0;
            for _ in 0..5_000 {
                assert!((-1.0..=1.0).contains(&command));
                command = steering_step(command, target, measured);
                measured = gain * -command;
            }
            assert!((measured - target).abs() <= STEERING_DEADBAND_DEG, "{measured}");
        }
    }

    #[test]
    fn drivetrain_command_is_duty_at_two_meters_per_second() {
        assert!((drivetrain_command(0.8) - 0.4).abs() < 1e-6);
        assert_eq!(drivetrain_command(5.0), 1.0);
        assert_eq!(drivetrain_command(-5.0), -1.0);
    }
}
