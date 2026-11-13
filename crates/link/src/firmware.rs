//! Firmware: what the vehicle runs, what firmware.ardupilot.org offers for
//! its board, and flashing an `.apj` through the ArduPilot bootloader over
//! USB (the protocol of ArduPilot's `Tools/scripts/uploader.py`).

use std::fmt;
use std::io::{self, Read, Write};
use std::time::Duration;

use crate::time::Instant;

use base64::Engine;
use serde::Deserialize;

// ---------- Versions ----------

/// A firmware version: 4.7.0-dev, 4.7.2, 4.7.3-beta1.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Version {
    pub major: u32,
    pub minor: u32,
    pub patch: u32,
    /// "dev", "beta1", "rc2"… empty for an official release.
    pub tag: String,
}

impl Version {
    /// Parses "4.7.2", "V4.7.3-beta1", or the version in "ArduCopter V4.7.0-dev (d0b31066)".
    pub fn parse(text: &str) -> Option<Self> {
        let word = text.split_whitespace().find(|w| w.trim_start_matches('V').starts_with(|c: char| c.is_ascii_digit()))?;
        let word = word.trim_start_matches('V');
        let (nums, tag) = word.split_once('-').unwrap_or((word, ""));
        let mut it = nums.split('.').map(|n| n.parse::<u32>());
        Some(Self { major: it.next()?.ok()?, minor: it.next().unwrap_or(Ok(0)).ok()?, patch: it.next().unwrap_or(Ok(0)).ok()?, tag: tag.to_owned() })
    }

    /// From AUTOPILOT_VERSION.flight_sw_version: major, minor, patch, type bytes.
    pub fn from_packed(v: u32) -> Self {
        let tag = match v & 0xff {
            0 => "dev",
            64 => "alpha",
            128 => "beta",
            192 => "rc",
            _ => "",
        };
        Self { major: v >> 24, minor: (v >> 16) & 0xff, patch: (v >> 8) & 0xff, tag: tag.to_owned() }
    }

    /// Release order: 4.7.0-dev < 4.7.0-beta1 < 4.7.0-rc1 < 4.7.0 < 4.7.1.
    fn rank(&self) -> (u32, u32, u32, u8, u32) {
        let t = self.tag.as_str();
        let stage = if t.is_empty() {
            4
        } else if t.starts_with("rc") {
            3
        } else if t.starts_with("beta") {
            2
        } else if t.starts_with("alpha") {
            1
        } else {
            0
        };
        let n = t.trim_start_matches(|c: char| c.is_ascii_alphabetic()).parse().unwrap_or(0);
        (self.major, self.minor, self.patch, stage, n)
    }

    pub fn is_newer_than(&self, other: &Version) -> bool {
        self.rank() > other.rank()
    }
}

impl fmt::Display for Version {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.patch)?;
        if !self.tag.is_empty() {
            write!(f, "-{}", self.tag)?;
        }
        Ok(())
    }
}

/// What the vehicle reports about its firmware.
#[derive(Debug, Clone, Default)]
pub struct FirmwareInfo {
    /// "ArduCopter V4.7.0-dev (d0b31066)".
    pub description: Option<String>,
    pub version: Option<Version>,
    pub git_hash: Option<String>,
    /// Board name as firmware.ardupilot.org names the build ("KakuteH7-bdshot").
    pub board: Option<String>,
    /// APJ board ID: the bootloader only accepts firmware built for it.
    pub board_id: Option<u32>,
    /// The banner lines (firmware, OS, board, outputs, IMUs).
    pub banner: Vec<String>,
    /// The flight controller's unique chip ID in hex, which tells one
    /// vehicle from another (it survives reflashing).
    pub uid: Option<String>,
    /// The chip ID as the banner (and every log) shows it, without spaces.
    pub serial: Option<String>,
}

/// The chip ID from AUTOPILOT_VERSION: `uid2` (ArduPilot fills in the MCU's
/// unique ID), else the 64-bit `uid`; `None` when both are zero.
pub fn chip_uid(uid: u64, uid2: &[u8]) -> Option<String> {
    let used = uid2.iter().rposition(|b| *b != 0).map_or(0, |i| i + 1);
    if used > 0 {
        Some(uid2[..used].iter().map(|b| format!("{b:02X}")).collect())
    } else {
        (uid != 0).then(|| format!("{uid:016X}"))
    }
}

impl FirmwareInfo {
    /// Fills in fields from one banner status text.
    pub fn add_banner_line(&mut self, text: &str) {
        let text = text.trim();
        if text.is_empty() || self.banner.iter().any(|l| l == text) {
            return;
        }
        self.banner.push(text.to_owned());
        if text.starts_with("Ardu") && text.contains(" V") {
            self.description = Some(text.to_owned());
            if let Some(v) = Version::parse(text) {
                self.version.get_or_insert(v);
            }
            if let Some(hash) = text.split('(').nth(1).map(|h| h.trim_end_matches(')').to_owned()) {
                self.git_hash.get_or_insert(hash);
            }
        }
        // ChibiOS boards report "<board name> <chip ID in hex>", the ID in one
        // block or in groups ("KakuteH7-bdshot 004D002D 33335101 38333837").
        if let Some(id) = ardutune_core::history::parse_board_line(text) {
            self.board = Some(id.board);
            self.serial = Some(id.serial);
        }
    }
}

// ---------- Release channels on firmware.ardupilot.org ----------

pub const SERVER: &str = "https://firmware.ardupilot.org";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    Stable,
    Beta,
    /// Built from the development branch.
    Latest,
}

impl Channel {
    pub const ALL: [Channel; 3] = [Channel::Stable, Channel::Beta, Channel::Latest];

    pub fn dir(self) -> &'static str {
        match self {
            Channel::Stable => "stable",
            Channel::Beta => "beta",
            Channel::Latest => "latest",
        }
    }

    pub fn from_dir(dir: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|c| c.dir() == dir)
    }

    /// Where Copter firmware for `board` lives in this channel.
    pub fn url(self, board: &str, file: &str) -> String {
        format!("{SERVER}/Copter/{}/{board}/{file}", self.dir())
    }
}

/// The version in a build's `git-version.txt` ("APMVERSION: ArduCopter V4.7.2").
pub fn version_from_git_file(text: &str) -> Option<Version> {
    text.lines().find_map(|l| l.strip_prefix("APMVERSION:")).and_then(Version::parse)
}

#[cfg(feature = "native")]
fn http_get(url: &str, limit: u64) -> Result<Vec<u8>, String> {
    let response = ureq::get(url).timeout(Duration::from_secs(30)).call().map_err(|e| match e {
        ureq::Error::Status(404, _) => format!("not found: {url}"),
        e => format!("{url}: {e}"),
    })?;
    let mut body = Vec::new();
    response.into_reader().take(limit).read_to_end(&mut body).map_err(|e| format!("{url}: {e}"))?;
    Ok(body)
}

/// The Copter version each channel offers for `board`; `None` where the
/// channel has no build for it. Needs internet access.
#[cfg(feature = "native")]
pub fn available(board: &str) -> Vec<(Channel, Result<Option<Version>, String>)> {
    Channel::ALL
        .into_iter()
        .map(|c| {
            let r = match http_get(&c.url(board, "git-version.txt"), 64 * 1024) {
                Ok(body) => Ok(version_from_git_file(&String::from_utf8_lossy(&body))),
                Err(e) if e.starts_with("not found") => Ok(None),
                Err(e) => Err(e),
            };
            (c, r)
        })
        .collect()
}

/// Downloads a channel's Copter firmware for `board`.
#[cfg(feature = "native")]
pub fn download(channel: Channel, board: &str) -> Result<Vec<u8>, String> {
    http_get(&channel.url(board, "arducopter.apj"), 64 * 1024 * 1024)
}

// ---------- .apj files ----------

#[derive(Deserialize)]
struct ApjJson {
    board_id: u32,
    #[serde(default)]
    board_revision: Option<u32>,
    image: String,
    #[serde(default)]
    image_size: Option<u64>,
    #[serde(default)]
    extf_image: Option<String>,
    #[serde(default)]
    extf_image_size: Option<u64>,
    #[serde(default)]
    summary: Option<String>,
    #[serde(default)]
    git_identity: Option<String>,
    #[serde(default)]
    description: Option<String>,
}

/// A firmware file: JSON with a base64, zlib-compressed flash image.
#[derive(Debug, Clone)]
pub struct Apj {
    pub board_id: u32,
    pub board_revision: Option<u32>,
    pub image: Vec<u8>,
    /// Board name ("KakuteH7-bdshot"), when given.
    pub summary: Option<String>,
    pub git_identity: Option<String>,
    pub description: Option<String>,
    /// Some boards also need an external-flash image, which isn't supported.
    pub has_external_flash: bool,
}

impl Apj {
    pub fn parse(bytes: &[u8]) -> Result<Self, String> {
        let json: ApjJson = serde_json::from_slice(bytes).map_err(|e| format!("not an ArduPilot .apj firmware file ({e})"))?;
        let image = unpack(&json.image, "firmware image")?;
        // Boards without external flash carry an empty compressed image here.
        let extf = match json.extf_image.as_deref().filter(|s| !s.is_empty()) {
            Some(text) => unpack(text, "external flash image")?.len(),
            None => 0,
        };
        if let Some(size) = json.image_size {
            if size != image.len() as u64 {
                return Err(format!("firmware image is {} bytes but the file says {size}", image.len()));
            }
        }
        Ok(Self {
            board_id: json.board_id,
            board_revision: json.board_revision,
            image,
            summary: json.summary,
            git_identity: json.git_identity,
            description: json.description,
            has_external_flash: extf > 0 || json.extf_image_size.unwrap_or(0) > 0,
        })
    }

    /// The image padded with 0xFF (erased flash) to a multiple of 4 bytes.
    pub fn padded_image(&self) -> Vec<u8> {
        let mut img = self.image.clone();
        while !img.len().is_multiple_of(4) {
            img.push(0xff);
        }
        img
    }
}

/// Base64, then zlib, as in `.apj` files.
fn unpack(text: &str, what: &str) -> Result<Vec<u8>, String> {
    let packed = base64::engine::general_purpose::STANDARD.decode(text.trim()).map_err(|e| format!("{what} is not valid base64 ({e})"))?;
    let mut out = Vec::new();
    flate2::read::ZlibDecoder::new(&packed[..]).read_to_end(&mut out).map_err(|e| format!("{what} doesn't decompress ({e})"))?;
    Ok(out)
}

// ---------- CRC ----------

const fn crc_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0xEDB8_8320 } else { c >> 1 };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

const CRC_TABLE: [u32; 256] = crc_table();

/// The bootloader's CRC-32: reflected 0xEDB88320, starting from `state`
/// (0 for a new CRC), without the usual final inversion.
pub fn crc32(data: &[u8], mut state: u32) -> u32 {
    for b in data {
        state = CRC_TABLE[((state ^ *b as u32) & 0xff) as usize] ^ (state >> 8);
    }
    state
}

/// The CRC the bootloader reports after programming `image` into a flash
/// of `flash_size` bytes: the image, then erased (0xFF) flash to the end.
pub fn flash_crc(image: &[u8], flash_size: usize) -> u32 {
    let mut state = crc32(image, 0);
    let mut i = image.len();
    while i < flash_size {
        state = crc32(&[0xff; 4], state);
        i += 4;
    }
    state
}

// ---------- Bootloader protocol ----------

const INSYNC: u8 = 0x12;
const EOC: u8 = 0x20;
const OK: u8 = 0x10;
const FAILED: u8 = 0x11;
const INVALID: u8 = 0x13;
const GET_SYNC: u8 = 0x21;
const GET_DEVICE: u8 = 0x22;
const CHIP_ERASE: u8 = 0x23;
const PROG_MULTI: u8 = 0x27;
const GET_CRC: u8 = 0x29;
const REBOOT: u8 = 0x30;
const INFO_BL_REV: u8 = 0x01;
const INFO_BOARD_ID: u8 = 0x02;
const INFO_BOARD_REV: u8 = 0x03;
const INFO_FLASH_SIZE: u8 = 0x04;
/// Bootloader protocol revisions this supports (CRC verification needs 3+).
const BL_REV_MIN: u32 = 3;
const BL_REV_MAX: u32 = 5;
const PROG_MULTI_MAX: usize = 252;
const ERASE_TIMEOUT: Duration = Duration::from_secs(30);
const REPLY_TIMEOUT: Duration = Duration::from_secs(2);

#[derive(Debug)]
pub enum FlashError {
    Io(io::Error),
    Timeout(&'static str),
    Protocol(String),
    /// The firmware was built for another board.
    WrongBoard {
        firmware: u32,
        board: u32,
    },
    TooBig {
        image: usize,
        flash: usize,
    },
    Unsupported(String),
    /// Programming finished but the flash doesn't hold the image.
    Verify {
        expected: u32,
        got: u32,
    },
}

impl fmt::Display for FlashError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlashError::Io(e) => write!(f, "USB connection to the bootloader failed: {e}"),
            FlashError::Timeout(what) => write!(f, "the bootloader didn't answer while {what}"),
            FlashError::Protocol(e) => write!(f, "bootloader error: {e}"),
            FlashError::WrongBoard { firmware, board } => {
                write!(f, "this firmware is for board ID {firmware} but the flight controller is board ID {board}; nothing was written")
            }
            FlashError::TooBig { image, flash } => {
                write!(f, "the firmware ({image} bytes) doesn't fit the flash ({flash} bytes); nothing was written")
            }
            FlashError::Unsupported(e) => write!(f, "{e}; nothing was written"),
            FlashError::Verify { expected, got } => {
                write!(f, "the written firmware doesn't verify (CRC {got:#010x}, expected {expected:#010x}); flash it again before powering off")
            }
        }
    }
}

impl std::error::Error for FlashError {}

impl From<io::Error> for FlashError {
    fn from(e: io::Error) -> Self {
        FlashError::Io(e)
    }
}

/// Where flashing is up to, for progress reports.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Stage {
    Erasing,
    Programming { done: usize, total: usize },
    Verifying,
    Rebooting,
}

/// The bootloader on the other end of `port` (a serial port whose reads
/// time out rather than block forever).
pub struct Bootloader<P> {
    port: P,
    pub bl_rev: u32,
    pub board_id: u32,
    pub board_rev: u32,
    pub flash_size: usize,
}

fn is_timeout(e: &io::Error) -> bool {
    matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock)
}

impl<P: Read + Write> Bootloader<P> {
    /// Syncs with the bootloader and reads what board it is.
    pub fn identify(mut port: P) -> Result<Self, FlashError> {
        let mut synced = false;
        for _ in 0..5 {
            discard_input(&mut port);
            port.write_all(&[GET_SYNC, EOC])?;
            port.flush()?;
            if read_sync(&mut port, Instant::now() + Duration::from_millis(500)).is_ok() {
                synced = true;
                break;
            }
        }
        if !synced {
            return Err(FlashError::Timeout("syncing"));
        }
        let mut bl = Self { port, bl_rev: 0, board_id: 0, board_rev: 0, flash_size: 0 };
        bl.bl_rev = bl.device_info(INFO_BL_REV)?;
        if !(BL_REV_MIN..=BL_REV_MAX).contains(&bl.bl_rev) {
            return Err(FlashError::Unsupported(format!("bootloader protocol revision {} isn't supported", bl.bl_rev)));
        }
        bl.board_id = bl.device_info(INFO_BOARD_ID)?;
        bl.board_rev = bl.device_info(INFO_BOARD_REV)?;
        bl.flash_size = bl.device_info(INFO_FLASH_SIZE)? as usize;
        Ok(bl)
    }

    fn device_info(&mut self, param: u8) -> Result<u32, FlashError> {
        self.port.write_all(&[GET_DEVICE, param, EOC])?;
        self.port.flush()?;
        let value = read_u32(&mut self.port, Instant::now() + REPLY_TIMEOUT)?;
        read_sync(&mut self.port, Instant::now() + REPLY_TIMEOUT)?;
        Ok(value)
    }

    /// Checks `apj` fits this board, then erases, programs, verifies by CRC
    /// and reboots into the new firmware. Nothing is erased if a check fails.
    pub fn flash(mut self, apj: &Apj, progress: &mut dyn FnMut(Stage)) -> Result<(), FlashError> {
        if apj.board_id != self.board_id {
            return Err(FlashError::WrongBoard { firmware: apj.board_id, board: self.board_id });
        }
        if apj.has_external_flash {
            return Err(FlashError::Unsupported("this firmware also needs external flash, which ArduTune can't write yet".to_owned()));
        }
        let image = apj.padded_image();
        if image.len() > self.flash_size {
            return Err(FlashError::TooBig { image: image.len(), flash: self.flash_size });
        }

        progress(Stage::Erasing);
        self.port.write_all(&[CHIP_ERASE, EOC])?;
        self.port.flush()?;
        read_sync(&mut self.port, Instant::now() + ERASE_TIMEOUT).map_err(|e| match e {
            FlashError::Timeout(_) => FlashError::Timeout("erasing"),
            e => e,
        })?;

        let total = image.len();
        for (i, chunk) in image.chunks(PROG_MULTI_MAX).enumerate() {
            let mut cmd = Vec::with_capacity(chunk.len() + 3);
            cmd.push(PROG_MULTI);
            cmd.push(chunk.len() as u8);
            cmd.extend_from_slice(chunk);
            cmd.push(EOC);
            self.port.write_all(&cmd)?;
            self.port.flush()?;
            read_sync(&mut self.port, Instant::now() + REPLY_TIMEOUT)?;
            progress(Stage::Programming { done: (i * PROG_MULTI_MAX + chunk.len()).min(total), total });
        }

        progress(Stage::Verifying);
        self.port.write_all(&[GET_CRC, EOC])?;
        self.port.flush()?;
        let got = read_u32(&mut self.port, Instant::now() + ERASE_TIMEOUT)?;
        read_sync(&mut self.port, Instant::now() + REPLY_TIMEOUT)?;
        let expected = flash_crc(&image, self.flash_size);
        if got != expected {
            return Err(FlashError::Verify { expected, got });
        }

        progress(Stage::Rebooting);
        self.port.write_all(&[REBOOT, EOC])?;
        self.port.flush()?;
        // The board resets now; a reply may or may not arrive.
        let _ = read_sync(&mut self.port, Instant::now() + Duration::from_millis(500));
        Ok(())
    }
}

fn discard_input<P: Read>(port: &mut P) {
    let mut buf = [0u8; 256];
    let deadline = Instant::now() + Duration::from_millis(100);
    while Instant::now() < deadline {
        match port.read(&mut buf) {
            Ok(0) => break,
            Ok(_) => {}
            Err(_) => break,
        }
    }
}

fn read_byte<P: Read>(port: &mut P, deadline: Instant) -> Result<u8, FlashError> {
    let mut b = [0u8; 1];
    loop {
        match port.read(&mut b) {
            Ok(1) => return Ok(b[0]),
            Ok(_) => {}
            Err(e) if is_timeout(&e) || e.kind() == io::ErrorKind::Interrupted => {}
            Err(e) => return Err(e.into()),
        }
        if Instant::now() >= deadline {
            return Err(FlashError::Timeout("waiting for a reply"));
        }
    }
}

/// Reads INSYNC followed by OK.
fn read_sync<P: Read>(port: &mut P, deadline: Instant) -> Result<(), FlashError> {
    let first = read_byte(port, deadline)?;
    if first != INSYNC {
        return Err(FlashError::Protocol(format!("expected INSYNC, got {first:#04x}")));
    }
    match read_byte(port, deadline)? {
        OK => Ok(()),
        INVALID => Err(FlashError::Protocol("the bootloader rejected a command as invalid".to_owned())),
        FAILED => Err(FlashError::Protocol("the bootloader reported a failure".to_owned())),
        other => Err(FlashError::Protocol(format!("unexpected reply {other:#04x}"))),
    }
}

fn read_u32<P: Read>(port: &mut P, deadline: Instant) -> Result<u32, FlashError> {
    let mut b = [0u8; 4];
    for x in &mut b {
        *x = read_byte(port, deadline)?;
    }
    Ok(u32::from_le_bytes(b))
}

/// A simulated bootloader for tests: answers the commands above and keeps
/// its flash in memory.
pub mod sim {
    use super::*;
    use std::collections::VecDeque;

    pub struct SimBootloader {
        pub board_id: u32,
        pub flash: Vec<u8>,
        pub erases: usize,
        pub rebooted: bool,
        /// Corrupt the byte at this offset when programming, to test verification.
        pub corrupt_at: Option<usize>,
        input: Vec<u8>,
        output: VecDeque<u8>,
        written: usize,
    }

    impl SimBootloader {
        pub fn new(board_id: u32, flash_size: usize) -> Self {
            Self {
                board_id,
                flash: vec![0xff; flash_size],
                erases: 0,
                rebooted: false,
                corrupt_at: None,
                input: Vec::new(),
                output: VecDeque::new(),
                written: 0,
            }
        }

        fn reply(&mut self, bytes: &[u8]) {
            self.output.extend(bytes);
            self.output.extend([INSYNC, OK]);
        }

        /// Runs every complete command in the input buffer.
        fn process(&mut self) {
            loop {
                let Some(&cmd) = self.input.first() else { return };
                let need = match cmd {
                    GET_SYNC | CHIP_ERASE | GET_CRC | REBOOT => 2,
                    GET_DEVICE => 3,
                    PROG_MULTI => match self.input.get(1) {
                        Some(&n) => n as usize + 3,
                        None => return,
                    },
                    _ => {
                        self.input.remove(0);
                        continue;
                    }
                };
                if self.input.len() < need {
                    return;
                }
                let frame: Vec<u8> = self.input.drain(..need).collect();
                match cmd {
                    GET_SYNC => self.reply(&[]),
                    GET_DEVICE => {
                        let v: u32 = match frame[1] {
                            INFO_BL_REV => 5,
                            INFO_BOARD_ID => self.board_id,
                            INFO_BOARD_REV => 0,
                            INFO_FLASH_SIZE => self.flash.len() as u32,
                            _ => 0,
                        };
                        self.reply(&v.to_le_bytes());
                    }
                    CHIP_ERASE => {
                        self.flash.iter_mut().for_each(|b| *b = 0xff);
                        self.erases += 1;
                        self.written = 0;
                        self.reply(&[]);
                    }
                    PROG_MULTI => {
                        let data = &frame[2..frame.len() - 1];
                        let start = self.written;
                        self.flash[start..start + data.len()].copy_from_slice(data);
                        if let Some(at) = self.corrupt_at.filter(|a| (start..start + data.len()).contains(a)) {
                            self.flash[at] ^= 0x01;
                        }
                        self.written += data.len();
                        self.reply(&[]);
                    }
                    GET_CRC => {
                        let crc = crc32(&self.flash, 0);
                        self.reply(&crc.to_le_bytes());
                    }
                    REBOOT => {
                        self.rebooted = true;
                        self.reply(&[]);
                    }
                    _ => {}
                }
            }
        }
    }

    impl Read for SimBootloader {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.output.is_empty() {
                return Err(io::Error::new(io::ErrorKind::TimedOut, "no data"));
            }
            let n = buf.len().min(self.output.len());
            for b in buf.iter_mut().take(n) {
                *b = self.output.pop_front().unwrap_or(0);
            }
            Ok(n)
        }
    }

    impl Write for SimBootloader {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.input.extend_from_slice(buf);
            self.process();
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
}

/// Writes an `.apj` for `image`, for tests and the simulated vehicle.
pub fn make_apj(board_id: u32, image: &[u8], summary: &str) -> Vec<u8> {
    let mut z = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::default());
    z.write_all(image).expect("in-memory write");
    let packed = z.finish().expect("in-memory write");
    serde_json::json!({
        "board_id": board_id,
        "magic": "APJFWv1",
        "description": "Firmware for a test board",
        "image": base64::engine::general_purpose::STANDARD.encode(packed),
        "image_size": image.len(),
        "summary": summary,
        "version": "0.1",
        "board_revision": 0,
        "git_identity": "test",
        // What real files carry on boards without external flash: an empty image.
        "extf_image": "eNoDAAAAAAE=",
        "extf_image_size": 0,
    })
    .to_string()
    .into_bytes()
}

#[cfg(test)]
mod tests {
    use super::sim::SimBootloader;
    use super::*;

    #[test]
    fn chip_ids() {
        let mut uid2 = [0u8; 18];
        uid2[..4].copy_from_slice(&[0x00, 0x3A, 0x00, 0x24]);
        assert_eq!(chip_uid(0, &uid2).as_deref(), Some("003A0024"));
        assert_eq!(chip_uid(0x1234, &[0; 18]).as_deref(), Some("0000000000001234"));
        assert_eq!(chip_uid(0, &[0; 18]), None);
    }

    #[test]
    fn versions_parse_and_order() {
        let dev = Version::parse("ArduCopter V4.7.0-dev (d0b31066)").unwrap();
        assert_eq!(dev.to_string(), "4.7.0-dev");
        let stable = Version::parse("V4.7.2").unwrap();
        let beta = Version::parse("4.7.3-beta1").unwrap();
        assert!(stable.is_newer_than(&dev));
        assert!(beta.is_newer_than(&stable));
        assert!(Version::parse("4.8.0-dev").unwrap().is_newer_than(&beta));
        assert!(!dev.is_newer_than(&dev));
        assert!(Version::parse("4.7.0").unwrap().is_newer_than(&Version::parse("4.7.0-rc2").unwrap()));
        assert_eq!(Version::from_packed(0x0407_0000).to_string(), "4.7.0-dev");
        assert_eq!(Version::from_packed(0x0407_02ff).to_string(), "4.7.2");
        let git = "commit fb77\nAuthor: x\n\n    Plane: version to 4.7.2\n\nAPMVERSION: ArduCopter V4.7.2\n";
        assert_eq!(version_from_git_file(git), Some(stable));
    }

    #[test]
    fn banner_gives_board_and_version() {
        let mut info = FirmwareInfo::default();
        for line in ["ArduCopter V4.7.0-dev (d0b31066)", "ChibiOS: 1fa4b8e0", "KakuteH7-bdshot 003A00243233510A33353533", "RCOut: DS600:1-4 PWM:5-8"] {
            info.add_banner_line(line);
        }
        assert_eq!(info.board.as_deref(), Some("KakuteH7-bdshot"));
        assert_eq!(info.version.unwrap().to_string(), "4.7.0-dev");
        assert_eq!(info.git_hash.as_deref(), Some("d0b31066"));

        // As a KakuteH7 on 4.6.3 sends it: the chip ID in groups.
        let mut info = FirmwareInfo::default();
        for line in ["ArduCopter V4.6.3 (92b0cd78)", "ChibiOS: 88b84600", "KakuteH7-bdshot 004D002D 33335101 38333837", "Frame: QUAD/X_REV"] {
            info.add_banner_line(line);
        }
        assert_eq!(info.board.as_deref(), Some("KakuteH7-bdshot"));
        assert_eq!(info.version.unwrap().to_string(), "4.6.3");
    }

    #[test]
    fn crc_matches_the_bootloader() {
        assert_eq!(CRC_TABLE[1], 0x7707_3096);
        assert_eq!(crc32(b"123456789", 0), 0x2dfd_2d88);
        let mut img: Vec<u8> = (0..10).collect();
        img.extend([0xff, 0xff]);
        assert_eq!(flash_crc(&img, 32), 0x6f65_5637);
    }

    #[test]
    fn apj_round_trips() {
        let image: Vec<u8> = (0..1001u32).map(|i| (i * 7) as u8).collect();
        let apj = Apj::parse(&make_apj(1048, &image, "KakuteH7-bdshot")).unwrap();
        assert_eq!((apj.board_id, apj.image.len()), (1048, 1001));
        assert_eq!(apj.padded_image().len(), 1004);
        assert_eq!(&apj.padded_image()[1001..], &[0xff; 3]);
        assert!(!apj.has_external_flash, "an empty external image is no external image");
        assert!(Apj::parse(b"{\"board_id\":1}").is_err());
        assert!(Apj::parse(b"not json").is_err());
    }

    /// Parses a real firmware file:
    /// `ARDUTUNE_APJ=arducopter.apj cargo test -p ardutune-link -- --ignored real_apj`
    #[test]
    #[ignore]
    fn real_apj() {
        let path = std::env::var("ARDUTUNE_APJ").expect("set ARDUTUNE_APJ");
        let apj = Apj::parse(&std::fs::read(path).unwrap()).unwrap();
        println!("board {} {:?}, {} bytes, {:?}", apj.board_id, apj.summary, apj.image.len(), apj.git_identity);
        assert!(!apj.has_external_flash);
        assert!(!apj.image.is_empty());
        // Flash it into a simulated bootloader with the KakuteH7's flash size.
        let mut sim = SimBootloader::new(apj.board_id, 1_703_936);
        Bootloader::identify(&mut sim).unwrap().flash(&apj, &mut |_| {}).unwrap();
        assert_eq!(&sim.flash[..apj.image.len()], &apj.image[..]);
        assert!(sim.rebooted);
    }

    #[test]
    fn flashes_verifies_and_reboots() {
        let image: Vec<u8> = (0..5000u32).map(|i| (i % 251) as u8).collect();
        let apj = Apj::parse(&make_apj(1048, &image, "Test")).unwrap();
        let mut sim = SimBootloader::new(1048, 64 * 1024);
        let bl = Bootloader::identify(&mut sim).unwrap();
        assert_eq!((bl.bl_rev, bl.board_id, bl.flash_size), (5, 1048, 65536));
        let mut stages = Vec::new();
        bl.flash(&apj, &mut |s| stages.push(s)).unwrap();
        assert_eq!(&sim.flash[..5000], &image[..]);
        assert!(sim.rebooted);
        assert_eq!(stages.first(), Some(&Stage::Erasing));
        assert!(stages.contains(&Stage::Programming { done: 5000, total: 5000 }));
        assert_eq!(stages.last(), Some(&Stage::Rebooting));
    }

    #[test]
    fn refuses_wrong_board_or_oversize_before_erasing() {
        let apj = Apj::parse(&make_apj(1048, &[1, 2, 3, 4], "Test")).unwrap();
        let mut sim = SimBootloader::new(9, 1024);
        let err = Bootloader::identify(&mut sim).unwrap().flash(&apj, &mut |_| {}).unwrap_err();
        assert!(matches!(err, FlashError::WrongBoard { firmware: 1048, board: 9 }));
        assert_eq!(sim.erases, 0);

        let big = Apj::parse(&make_apj(9, &vec![0u8; 2048], "Test")).unwrap();
        let err = Bootloader::identify(&mut sim).unwrap().flash(&big, &mut |_| {}).unwrap_err();
        assert!(matches!(err, FlashError::TooBig { .. }));
        assert_eq!(sim.erases, 0);
    }

    #[test]
    fn detects_a_bad_write() {
        let apj = Apj::parse(&make_apj(9, &vec![0x5a; 600], "Test")).unwrap();
        let mut sim = SimBootloader::new(9, 4096);
        sim.corrupt_at = Some(300);
        let err = Bootloader::identify(&mut sim).unwrap().flash(&apj, &mut |_| {}).unwrap_err();
        assert!(matches!(err, FlashError::Verify { .. }), "{err}");
        assert!(!sim.rebooted, "doesn't reboot into a bad image");
    }
}
