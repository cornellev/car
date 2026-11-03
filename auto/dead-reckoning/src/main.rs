use serde::Serialize;
use std::f64::consts::PI;
use std::time::{Duration, Instant};
use tokio::time::{interval, MissedTickBehavior};

const LEFT_RPM: &str = "cev/sensors/wheels/left_rpm";
const RIGHT_RPM: &str = "cev/sensors/wheels/right_rpm";
const STEERING: &str = "cev/sensors/steering/steering_angle";
const POSE: &str = "cev/localization/pose";

// Replace with measured vehicle parameters.
const WHEEL_RADIUS: f64 = 0.10; // meters //TODO
const WHEELBASE: f64 = 0.50;    // meters /?TODO

const PUBLISH_HZ: u64 = 50;
const SENSOR_TIMEOUT: Duration = Duration::from_millis(200); //SET TIMEOUT

#[derive(Debug, Clone, Copy, Serialize)]
struct Pose {
    x: f64,
    y: f64,
    z: f64,
    heading: f64,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            heading: 0.0,
        }
    }
}

#[derive(Clone, Copy)]
struct Measurement {
    value: f64,
    received_at: Instant,
}

#[derive(Default)]
struct Sensors {
    left: Option<Measurement>,
    right: Option<Measurement>,
    steering: Option<Measurement>,
}

impl Sensors {
    fn ready(&self, now: Instant) -> Option<(f64, f64, f64)> {
        let l = self.left?;
        let r = self.right?;
        let s = self.steering?;

        for m in [l, r, s] {
            if now.duration_since(m.received_at) > SENSOR_TIMEOUT {
                return None;
            }
        }

        Some((l.value, r.value, s.value))
    }
}

fn decode_float(bytes: &[u8]) -> Option<f64> {
    let data: [u8; 4] = bytes.try_into().ok()?;
    let value = f32::from_le_bytes(data) as f64;

    value.is_finite().then_some(value)
}

fn normalize_angle(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}

fn integrate(
    pose: &mut Pose,
    left_rps: f64,
    right_rps: f64,
    steering: f64,
    dt: f64,
) {
    // Mean rear-wheel longitudinal velocity, m/s.
    let velocity = PI * WHEEL_RADIUS * (left_rps + right_rps);

    // Bicycle model yaw rate, rad/s.
    let yaw_rate = velocity * steering.tan() / WHEELBASE;

    let delta_heading = yaw_rate * dt;
    let midpoint_heading = pose.heading + delta_heading / 2.0;

    pose.x += velocity * dt * midpoint_heading.cos();
    pose.y += velocity * dt * midpoint_heading.sin();
    pose.heading = normalize_angle(pose.heading + delta_heading);
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let session = zenoh::open(zenoh::Config::default()).await?;

    let left_sub = session.declare_subscriber(LEFT_RPM).await?;
    let right_sub = session.declare_subscriber(RIGHT_RPM).await?;
    let steering_sub = session.declare_subscriber(STEERING).await?;

    let publisher = session.declare_publisher(POSE).await?;

    let mut sensors = Sensors::default();
    let mut pose = Pose::default();

    let mut ticker = interval(Duration::from_secs_f64(
        1.0 / PUBLISH_HZ as f64
    ));
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    let mut last_update = Instant::now();

    loop {
        tokio::select! {
            result = left_sub.recv_async() => {
                let sample = result?;
                let bytes = sample.payload().to_bytes();

                if let Some(value) = decode_float(bytes.as_ref()) {
                    sensors.left = Some(Measurement {
                        value,
                        received_at: Instant::now(),
                    });
                }
            }

            result = right_sub.recv_async() => {
                let sample = result?;
                let bytes = sample.payload().to_bytes();

                if let Some(value) = decode_float(bytes.as_ref()) {
                    sensors.right = Some(Measurement {
                        value,
                        received_at: Instant::now(),
                    });
                }
            }

            result = steering_sub.recv_async() => {
                let sample = result?;
                let bytes = sample.payload().to_bytes();

                if let Some(value) = decode_float(bytes.as_ref()) {
                    sensors.steering = Some(Measurement {
                        value,
                        received_at: Instant::now(),
                    });
                }
            }

            _ = ticker.tick() => {
                let now = Instant::now();
                let dt = now.duration_since(last_update).as_secs_f64();
                last_update = now;

                // Only integrate when all measurements are fresh.
                if let Some((left, right, steering)) = sensors.ready(now)
                {
                    integrate(
                        &mut pose,
                        left,
                        right,
                        steering,
                        dt,
                    );
                }

                let json = serde_json::to_string(&pose)?;
                publisher.put(json).await?;
            }
        }
    }
}
