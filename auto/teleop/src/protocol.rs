//! Serial protocol shared with the merceves firmware (elec/src/merceves/main.cpp).
//!
//! Every frame is COBS-encoded and ends in a 0x00 byte. Decoded, a frame is
//! `[type u8][payload]`. All multi-byte values are little-endian.
//! There's no checksum: USB CRC-checks every packet and resends bad ones.
//!
//! - jetson -> pico, `MSG_DRIVE`: f32 steering position [-1, 1], f32 drivetrain speed [-1, 1]
//! - pico -> jetson, `MSG_TELEMETRY`: f32 left rps, f32 right rps, i32 steering pulses relative to homed center, u16 encoder raw, u8 status, u8 read_ok, f32 steering angle in degrees (positive left)

pub const MSG_DRIVE: u8 = 0x01;
pub const MSG_TELEMETRY: u8 = 0x02;

/// Encodes a drive command, ready to write to the port (trailing 0x00 included).
pub fn encode_drive(steering: f32, drivetrain: f32) -> Vec<u8> {
    let mut frame = vec![MSG_DRIVE];
    frame.extend_from_slice(&steering.to_le_bytes());
    frame.extend_from_slice(&drivetrain.to_le_bytes());

    let mut out = cobs::encode_vec(&frame);
    out.push(0x00);
    out
}

/// A successful AS5600 I2C sample. Magnetic warnings are preserved in `status`.
#[derive(Debug, PartialEq)]
pub struct EncoderReading {
    pub raw: u16,
    pub status: u8,
}

#[derive(Debug, PartialEq)]
pub struct Telemetry {
    pub left_rps: f32,
    pub right_rps: f32,
    pub steering_pulses: i32,
    /// None before homing or when encoder angle is unavailable.
    pub steering_degrees: Option<f32>,
    /// None when the latest I2C transaction failed; never a stale previous reading.
    pub encoder: Option<EncoderReading>,
}

/// Decode bytes before a frame's trailing 0x00 delimiter.
pub fn decode_telemetry(encoded: &[u8]) -> Option<Telemetry> {
    let frame = cobs::decode_vec(encoded).ok()?;
    if frame.len() != 21 || frame[0] != MSG_TELEMETRY {
        return None;
    }
    let raw = u16::from_le_bytes(frame[13..15].try_into().ok()?);
    let encoder = match frame[16] {
        0 => None,
        1 if raw < 4096 => Some(EncoderReading {
            raw,
            status: frame[15],
        }),
        _ => return None,
    };
    Some(Telemetry {
        left_rps: f32::from_le_bytes(frame[1..5].try_into().ok()?),
        right_rps: f32::from_le_bytes(frame[5..9].try_into().ok()?),
        steering_pulses: i32::from_le_bytes(frame[9..13].try_into().ok()?),
        steering_degrees: {
            let degrees = f32::from_le_bytes(frame[17..21].try_into().ok()?);
            degrees.is_finite().then_some(degrees)
        },
        encoder,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    // bytes produced by the firmware's own encoder for the same command
    #[test]
    fn drive_matches_firmware() {
        assert_eq!(
            encode_drive(-0.5, 0.25),
            [
                0x02, 0x01, 0x01, 0x01, 0x02, 0xbf, 0x01, 0x03, 0x80, 0x3e, 0x00
            ]
        );
    }

    #[test]
    fn telemetry_round_trip() {
        let mut frame = vec![MSG_TELEMETRY];
        frame.extend_from_slice(&120.5f32.to_le_bytes());
        frame.extend_from_slice(&(-3.0f32).to_le_bytes());
        frame.extend_from_slice(&(-1234i32).to_le_bytes());
        frame.extend_from_slice(&1596u16.to_le_bytes());
        frame.extend_from_slice(&[0x33, 1]);
        frame.extend_from_slice(&(-12.5f32).to_le_bytes());
        let encoded = cobs::encode_vec(&frame);

        assert_eq!(
            decode_telemetry(&encoded),
            Some(Telemetry {
                left_rps: 120.5,
                right_rps: -3.0,
                steering_pulses: -1234,
                steering_degrees: Some(-12.5),
                encoder: Some(EncoderReading {
                    raw: 1596,
                    status: 0x33
                }),
            })
        );
    }

    #[test]
    fn telemetry_failed_encoder_keeps_other_measurements() {
        // Literal wire layout: left=120.5, right=-3, pulses=-1234, failed I2C.
        let frame = [
            2, 0, 0, 241, 66, 0, 0, 64, 192, 46, 251, 255, 255, 0, 0, 0, 0, 0, 0, 192, 127,
        ];
        assert_eq!(
            decode_telemetry(&cobs::encode_vec(&frame)),
            Some(Telemetry {
                left_rps: 120.5,
                right_rps: -3.0,
                steering_pulses: -1234,
                steering_degrees: None,
                encoder: None,
            })
        );
    }

    #[test]
    fn telemetry_accepts_zero_and_max_raw_even_with_magnet_warnings() {
        for raw in [0u16, 4095] {
            let mut frame = [0u8; 21];
            frame[0] = MSG_TELEMETRY;
            frame[13..15].copy_from_slice(&raw.to_le_bytes());
            frame[15] = 0x33;
            frame[16] = 1;
            assert_eq!(
                decode_telemetry(&cobs::encode_vec(&frame)).unwrap().encoder,
                Some(EncoderReading { raw, status: 0x33 })
            );
        }
    }

    #[test]
    fn telemetry_rejects_invalid_encoder_fields() {
        let mut frame = [0u8; 21];
        frame[0] = MSG_TELEMETRY;
        frame[16] = 1;
        frame[13..15].copy_from_slice(&4096u16.to_le_bytes());
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
        frame[13..15].copy_from_slice(&0u16.to_le_bytes());
        frame[16] = 2;
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
    }

    #[test]
    fn telemetry_rejects_previous_pulse_only_layout() {
        let mut frame = [0u8; 13];
        frame[0] = MSG_TELEMETRY;
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
    }

    #[test]
    fn telemetry_rejects_old_layout() {
        let mut frame = vec![MSG_TELEMETRY];
        frame.extend_from_slice(&120.5f32.to_le_bytes());
        frame.extend_from_slice(&3.0f32.to_le_bytes());
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
    }

    #[test]
    fn telemetry_rejects_previous_encoder_layout() {
        let mut frame = [0u8; 17];
        frame[0] = MSG_TELEMETRY;
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
    }

    // a frame that lost bytes in transit must not decode as telemetry
    #[test]
    fn telemetry_rejects_truncated_frame() {
        let mut frame = vec![MSG_TELEMETRY];
        frame.extend_from_slice(&120.5f32.to_le_bytes());
        frame.extend_from_slice(&[0x11; 2]);
        let encoded = cobs::encode_vec(&frame);

        assert_eq!(decode_telemetry(&encoded), None);
    }
}
