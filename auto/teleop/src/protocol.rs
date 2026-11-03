//! Serial protocol shared with the merceves firmware (elec/src/merceves/main.cpp).
//!
//! Every frame is COBS-encoded and ends in a 0x00 byte. Decoded, a frame is
//! `[type u8][payload]`. All multi-byte values are little-endian.
//! There's no checksum: USB CRC-checks every packet and resends bad ones.
//!
//! - jetson -> pico, `MSG_DRIVE`: f32 steering position [-1, 1], f32 drivetrain speed [-1, 1]
//! - pico -> jetson, `MSG_TELEMETRY`: f32 left rot/s, f32 right rot/s, i32 encoder raw, i32 encoder wrap

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

/// Decodes one telemetry frame into (left rot/s, right rot/s, encoder raw, encoder wrap).
/// Pass the bytes before a 0x00, without it.
pub fn decode_telemetry(encoded: &[u8]) -> Option<(f32, f32, i32, i32)> {
    let frame = cobs::decode_vec(encoded).ok()?;
    if frame.len() != 17 || frame[0] != MSG_TELEMETRY {
        return None;
    }

    let left = f32::from_le_bytes(frame[1..5].try_into().ok()?);
    let right = f32::from_le_bytes(frame[5..9].try_into().ok()?);
    let raw = i32::from_le_bytes(frame[9..13].try_into().ok()?);
    let wrap = i32::from_le_bytes(frame[13..17].try_into().ok()?);
    Some((left, right, raw, wrap))
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
        frame.extend_from_slice(&1.5f32.to_le_bytes());
        frame.extend_from_slice(&(-0.25f32).to_le_bytes());
        frame.extend_from_slice(&(-1234i32).to_le_bytes());
        frame.extend_from_slice(&2i32.to_le_bytes());
        let encoded = cobs::encode_vec(&frame);

        assert_eq!(decode_telemetry(&encoded), Some((1.5, -0.25, -1234, 2)));
    }

    #[test]
    fn telemetry_rejects_rpm_and_pulse_layout() {
        let mut frame = vec![MSG_TELEMETRY];
        frame.extend_from_slice(&120.5f32.to_le_bytes());
        frame.extend_from_slice(&3.0f32.to_le_bytes());
        frame.extend_from_slice(&900i32.to_le_bytes());
        assert_eq!(decode_telemetry(&cobs::encode_vec(&frame)), None);
    }

    #[test]
    fn telemetry_rejects_old_layout() {
        let mut frame = vec![MSG_TELEMETRY];
        frame.extend_from_slice(&120.5f32.to_le_bytes());
        frame.extend_from_slice(&3.0f32.to_le_bytes());
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
