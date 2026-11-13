//! MAVLink frames from a raw byte stream, for transports that only move
//! bytes (the web app's Web Serial).

use std::io::Cursor;

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::{MavHeader, MavlinkReader};

use crate::{GCS_COMPONENT_ID, GCS_SYSTEM_ID};

const V2_MAGIC: u8 = 0xFD;
const V1_MAGIC: u8 = 0xFE;

/// Collects bytes and hands out the complete messages in them; noise and
/// corrupt frames are skipped.
#[derive(Default)]
pub struct Decoder {
    buf: Vec<u8>,
}

impl Decoder {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// The next complete message, if one has arrived.
    pub fn next_message(&mut self) -> Option<(MavHeader, MavMessage)> {
        loop {
            // Up to the next frame start.
            let start = self.buf.iter().position(|b| *b == V2_MAGIC || *b == V1_MAGIC)?;
            self.buf.drain(..start);
            let total = match self.buf[0] {
                // Only the signing bit is defined in the incompatibility flags:
                // anything else is noise that happens to look like a start.
                V2_MAGIC if self.buf.len() >= 3 && self.buf[2] & !1 != 0 => {
                    self.buf.drain(..1);
                    continue;
                }
                V2_MAGIC if self.buf.len() >= 3 => {
                    let signed = self.buf[2] & 1 != 0;
                    10 + self.buf[1] as usize + 2 + if signed { 13 } else { 0 }
                }
                V1_MAGIC if self.buf.len() >= 2 => 6 + self.buf[1] as usize + 2,
                _ => return None,
            };
            if self.buf.len() < total {
                return None;
            }
            let mut reader = MavlinkReader::new(Cursor::new(&self.buf[..total]));
            match reader.read_any_message::<MavMessage>() {
                Ok(m) => {
                    self.buf.drain(..total);
                    return Some(m);
                }
                // Not a frame after all, or one we can't read: look further.
                Err(_) => {
                    self.buf.drain(..1);
                }
            }
        }
    }
}

/// One message as MAVLink 2 bytes from our GCS ids.
pub fn encode(sequence: u8, msg: &MavMessage) -> Vec<u8> {
    let header = MavHeader { system_id: GCS_SYSTEM_ID, component_id: GCS_COMPONENT_ID, sequence };
    let mut out = Vec::new();
    let _ = mavlink::write_v2_msg(&mut out, header, msg);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gcs_heartbeat;

    #[test]
    fn splits_a_stream_with_noise_and_partial_frames() {
        let a = encode(1, &gcs_heartbeat());
        let b = encode(2, &gcs_heartbeat());
        let mut stream = vec![0x00, 0xFD, 0x42]; // noise, including a false start
        stream.extend(&a);
        stream.extend(&b);
        let mut d = Decoder::default();
        // In two pieces, the cut inside the second frame.
        let cut = stream.len() - 5;
        d.push(&stream[..cut]);
        let first = d.next_message().expect("first frame");
        assert_eq!(first.0.sequence, 1);
        assert!(d.next_message().is_none(), "second frame not complete yet");
        d.push(&stream[cut..]);
        let second = d.next_message().expect("second frame");
        assert_eq!((second.0.sequence, second.0.system_id), (2, GCS_SYSTEM_ID));
        assert!(matches!(second.1, MavMessage::HEARTBEAT(_)));
        assert!(d.next_message().is_none());
    }
}
