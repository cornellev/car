//! Publish gamepad sticks for teleop: left stick x steers, right stick y drives (up is forward).
//! The triggers aren't used: on our controller, LT wraps from 255 back to 0 when fully pressed.
//!
//! usage: gamepad
//! Publishes JSON `{"steering_angle","speed"}` on `cev/commands/control` at 50 Hz.
//! No gamepad keeps publishing zeros; teleop holds the last sample.

use std::time::Duration;

use gilrs::{Axis, EventType, Gilrs};
use serde::Serialize;

const SEND_PERIOD: Duration = Duration::from_millis(20); // match teleop's serial rate
const COMMAND_KEY: &str = "cev/commands/control"; // teleop's config::ACKERMANN_KEY

const STICK_DEADZONE: f32 = 0.1;
// Same units teleop consumes: radians and m/s. Full stick is the 11 deg limit
// and 1.2 m/s, which is 0.6 of the 2 m/s full-scale duty.
const MAX_STEERING_RAD: f32 = 11.0_f32.to_radians();
const MAX_SPEED_MPS: f32 = 1.2;

#[derive(Serialize)]
struct Command {
    steering_angle: f64,
    speed: f64,
}

// zero inside the deadzone, then rescaled so the output still reaches 1.0 at full stick
fn deadzone(value: f32) -> f32 {
    if value.abs() < STICK_DEADZONE {
        0.0
    } else {
        (value - STICK_DEADZONE * value.signum()) / (1.0 - STICK_DEADZONE)
    }
}

fn axes_to_command(stick_x: f32, stick_y: f32) -> Command {
    Command {
        steering_angle: (deadzone(stick_x) * MAX_STEERING_RAD) as f64,
        speed: (deadzone(stick_y) * MAX_SPEED_MPS) as f64,
    }
}

fn read_command(gilrs: &mut Gilrs) -> Command {
    while let Some(event) = gilrs.next_event() {
        match event.event {
            EventType::Connected => {
                eprintln!("gamepad connected: {}", gilrs.gamepad(event.id).name())
            }
            EventType::Disconnected => eprintln!("gamepad disconnected, stopping"),
            _ => {}
        }
    }

    match gilrs.gamepads().next() {
        Some((_, pad)) => {
            axes_to_command(pad.value(Axis::LeftStickX), pad.value(Axis::RightStickY))
        }
        None => Command {
            steering_angle: 0.0,
            speed: 0.0,
        },
    }
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut pads = Gilrs::new().expect("failed to start gamepad input");
    for (_, pad) in pads.gamepads() {
        eprintln!("gamepad: {}", pad.name());
    }

    let session = zenoh::open(zenoh::Config::default()).await.unwrap();
    let publisher = session.declare_publisher(COMMAND_KEY).await?;

    loop {
        let payload = serde_json::to_vec(&read_command(&mut pads)).expect("command json");
        if let Err(e) = publisher.put(payload).await {
            eprintln!("publish failed: {e}");
        }
        tokio::time::sleep(SEND_PERIOD).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sticks_scale_into_the_teleop_command() {
        let stopped = axes_to_command(0.05, -0.05);
        assert_eq!(stopped.steering_angle, 0.0);
        assert_eq!(stopped.speed, 0.0);

        let full = axes_to_command(1.0, 1.0);
        assert!((full.steering_angle - MAX_STEERING_RAD as f64).abs() < 1e-9);
        assert!((full.speed - 1.2).abs() < 1e-6);

        assert_eq!(
            serde_json::to_string(&axes_to_command(0.0, 0.0)).unwrap(),
            r#"{"steering_angle":0.0,"speed":0.0}"#
        );
    }
}
