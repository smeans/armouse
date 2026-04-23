// Session-based positioning using RELATIVE mouse deltas.
//
// Absolute coordinates (Coordinate::Abs) on Windows SendInput use virtual-
// desktop normalization that breaks down on DPI-scaled secondary monitors
// (e.g. XReal AR virtual displays). Relative deltas work in raw pixels on
// whichever monitor the cursor is currently on, so they behave correctly
// across mixed-DPI setups.
//
// Algorithm per session:
//   - capture the cursor's starting position
//   - integrate gyro to accumulate yaw/pitch angles in radians
//   - each tick, compute desired = start + angle * sensitivity
//     and send a RELATIVE delta of (desired - current_location).
//
// Axis mapping (confirmed empirically):
//   gyro[0] = pitch (negative = looking down)
//   gyro[1] = roll  (unused)
//   gyro[2] = yaw   (negative = looking left)

use std::time::{Duration, Instant};

use enigo::{Coordinate, Enigo, Mouse, Settings};
use xreal_one_driver::XrealOne;

#[cfg(windows)]
fn enable_per_monitor_dpi_awareness() {
    use windows::Win32::UI::HiDpi::{
        SetProcessDpiAwarenessContext, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2,
    };
    // Safe: single call, well-defined Windows API. If it fails (e.g. already
    // set by a manifest), we continue — at worst behaviour is unchanged.
    unsafe {
        let _ = SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    }
}

// pixels per radian. Sign chosen so the cursor tracks head direction:
//   look right -> cursor right, look down -> cursor down.
// Flip the sign on either constant if your setup moves the wrong way.
const SENSITIVITY_X: f32 = -2000.0;
const SENSITIVITY_Y: f32 = 2000.0;

// Throttle cursor updates so we don't flood SendInput at the 1 kHz IMU rate
// (which starves out the physical mouse on Windows).
const MOUSE_UPDATE_INTERVAL: Duration = Duration::from_millis(8); // ~125 Hz

const DEBUG_INTERVAL: Duration = Duration::from_millis(200);

struct Session {
    start_x: i32,
    start_y: i32,
    yaw_angle: f32,   // integrated gyro[2] (radians), positive = looking right
    pitch_angle: f32, // integrated gyro[0] (radians), positive = looking up
}

impl Session {
    fn new(enigo: &Enigo) -> Self {
        let (x, y) = enigo.location().expect("failed to get mouse location");
        Self {
            start_x: x,
            start_y: y,
            yaw_angle: 0.0,
            pitch_angle: 0.0,
        }
    }

    fn desired(&self) -> (i32, i32) {
        let dx = self.yaw_angle * SENSITIVITY_X;
        let dy = self.pitch_angle * SENSITIVITY_Y;
        (self.start_x + dx.round() as i32, self.start_y + dy.round() as i32)
    }
}

fn main() {
    #[cfg(windows)]
    enable_per_monitor_dpi_awareness();

    let mut xreal = XrealOne::new().expect("failed to connect to XReal One glasses");
    let mut enigo = Enigo::new(&Settings::default()).expect("failed to init enigo");

    let mut session = Session::new(&enigo);
    println!(
        "connected. session started at ({}, {}). moving cursor from head motion. Ctrl-C to exit.",
        session.start_x, session.start_y,
    );

    let mut prev_ts: Option<u64> = None;
    let mut last_move = Instant::now() - MOUSE_UPDATE_INTERVAL;

    // Debug counters
    let mut samples: u32 = 0;
    let mut last_gyro = [0.0f32; 3];
    let mut last_debug = Instant::now();

    loop {
        let imu = xreal.next().expect("failed to read IMU sample");
        samples += 1;
        last_gyro = imu.gyro;

        // Timestamps are in microseconds (~1 ms between samples).
        let dt = match prev_ts {
            Some(prev) => (imu.timestamp.saturating_sub(prev) as f32) * 1e-6,
            None => 0.0,
        };
        prev_ts = Some(imu.timestamp);

        // Integrate every valid sample so the tracked angle stays accurate
        // even when we don't emit a mouse move this tick.
        if dt > 0.0 && dt <= 0.1 {
            session.yaw_angle += imu.gyro[2] * dt;
            session.pitch_angle += imu.gyro[0] * dt;
        }

        let now = Instant::now();
        if now.duration_since(last_move) >= MOUSE_UPDATE_INTERVAL {
            last_move = now;

            let (desired_x, desired_y) = session.desired();
            let (cur_x, cur_y) = enigo
                .location()
                .expect("failed to get current mouse location");

            let dx = desired_x - cur_x;
            let dy = desired_y - cur_y;

            if dx != 0 || dy != 0 {
                enigo
                    .move_mouse(dx, dy, Coordinate::Rel)
                    .expect("failed to move mouse");
            }
        }

        if now.duration_since(last_debug) >= DEBUG_INTERVAL {
            last_debug = now;
            let (dx, dy) = session.desired();
            let (cx, cy) = enigo.location().unwrap_or((-999, -999));
            println!(
                "samples={:>4}  gyro=[{:>7.4},{:>7.4},{:>7.4}]  angle=[yaw={:>7.3},pitch={:>7.3}]  desired=({:>5},{:>5})  actual=({:>5},{:>5})",
                samples, last_gyro[0], last_gyro[1], last_gyro[2],
                session.yaw_angle, session.pitch_angle, dx, dy, cx, cy,
            );
            samples = 0;
        }
    }
}
