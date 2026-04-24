// armouse: head-motion mouse control for XReal One glasses on Windows.
//
// Milestone 4: hold Right Alt to activate. While held, head motion drives
// the cursor using the session-based absolute-via-relative-delta algorithm
// from milestone 3. Release Right Alt to freeze the cursor.
//
// Threading model:
//   - IMU reader thread: loops xreal.next() and pushes samples through an
//     mpsc channel. Keeps the TCP socket drained even while we're idle.
//   - Keyboard hook thread: installs a WH_KEYBOARD_LL hook and runs a
//     Windows message pump. The hook callback flips an AtomicBool for
//     Right Alt. Low-level hooks observe input without consuming it, so
//     Right Alt still works normally for other apps (unlike RegisterHotKey).
//   - Main thread: owns Enigo. Every tick it checks the atomic, drains the
//     IMU channel, integrates gyro into the current Session, and nudges
//     the cursor toward the desired position. On the false->true edge it
//     starts a new Session (recapturing cursor origin, zeroing angles).
//
// Axis mapping (empirical):
//   gyro[0] = pitch (negative = looking down)
//   gyro[1] = roll  (unused)
//   gyro[2] = yaw   (negative = looking left)

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, TryRecvError};
use std::thread;
use std::time::{Duration, Instant};

use enigo::{Coordinate, Enigo, Mouse, Settings};
use xreal_one_driver::{XOImu, XrealOne};

// pixels per radian. Sign chosen so the cursor tracks head direction:
//   look right -> cursor right, look down -> cursor down.
// Flip the sign on either constant if your setup moves the wrong way.
const SENSITIVITY_X: f32 = -2000.0;
const SENSITIVITY_Y: f32 = 2000.0;

// Throttle cursor updates so we don't flood SendInput at the 1 kHz IMU rate
// (which starves out the physical mouse on Windows).
const MOUSE_UPDATE_INTERVAL: Duration = Duration::from_millis(8); // ~125 Hz

const DEBUG_INTERVAL: Duration = Duration::from_millis(500);

// Shared state between the keyboard hook thread and main thread.
static RIGHT_ALT_DOWN: AtomicBool = AtomicBool::new(false);

#[cfg(windows)]
fn enable_per_monitor_dpi_awareness() {
    use windows::Win32::UI::HiDpi::{
        SetProcessDpiAwarenessContext, DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2,
    };
    unsafe {
        let _ = SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    }
}

#[cfg(windows)]
mod kb_hook {
    use super::RIGHT_ALT_DOWN;
    use std::sync::atomic::Ordering;
    use windows::Win32::Foundation::{LPARAM, LRESULT, WPARAM};
    use windows::Win32::UI::Input::KeyboardAndMouse::VK_RMENU;
    use windows::Win32::UI::WindowsAndMessaging::{
        CallNextHookEx, DispatchMessageW, GetMessageW, SetWindowsHookExW, TranslateMessage,
        HHOOK, KBDLLHOOKSTRUCT, MSG, WH_KEYBOARD_LL, WM_KEYDOWN, WM_KEYUP, WM_SYSKEYDOWN,
        WM_SYSKEYUP,
    };

    unsafe extern "system" fn low_level_keyboard_proc(
        code: i32,
        wparam: WPARAM,
        lparam: LPARAM,
    ) -> LRESULT {
        if code >= 0 {
            let kbd = &*(lparam.0 as *const KBDLLHOOKSTRUCT);
            if kbd.vkCode == VK_RMENU.0 as u32 {
                match wparam.0 as u32 {
                    WM_KEYDOWN | WM_SYSKEYDOWN => {
                        RIGHT_ALT_DOWN.store(true, Ordering::Release);
                    }
                    WM_KEYUP | WM_SYSKEYUP => {
                        RIGHT_ALT_DOWN.store(false, Ordering::Release);
                    }
                    _ => {}
                }
            }
        }
        // Always pass the event through. We're observing, not consuming.
        CallNextHookEx(HHOOK::default(), code, wparam, lparam)
    }

    /// Installs the hook and runs a message pump. Never returns.
    pub fn run() -> ! {
        unsafe {
            let _hook = SetWindowsHookExW(WH_KEYBOARD_LL, Some(low_level_keyboard_proc), None, 0)
                .expect("failed to install WH_KEYBOARD_LL hook");
            // Message pump — required for low-level hooks to be called.
            let mut msg = MSG::default();
            while GetMessageW(&mut msg, None, 0, 0).as_bool() {
                let _ = TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
        }
        // GetMessageW only returns FALSE on WM_QUIT, which we never post.
        loop {
            std::thread::park();
        }
    }
}

struct Session {
    start_x: i32,
    start_y: i32,
    yaw_angle: f32,   // integrated gyro[2] (radians), positive = looking right
    pitch_angle: f32, // integrated gyro[0] (radians), positive = looking up
    prev_ts: Option<u64>,
}

impl Session {
    fn new(enigo: &Enigo) -> Self {
        let (x, y) = enigo.location().expect("failed to get mouse location");
        Self {
            start_x: x,
            start_y: y,
            yaw_angle: 0.0,
            pitch_angle: 0.0,
            prev_ts: None,
        }
    }

    fn integrate(&mut self, imu: &XOImu) {
        let dt = match self.prev_ts {
            Some(prev) => (imu.timestamp.saturating_sub(prev) as f32) * 1e-6,
            None => 0.0,
        };
        self.prev_ts = Some(imu.timestamp);
        if dt > 0.0 && dt <= 0.1 {
            self.yaw_angle += imu.gyro[2] * dt;
            self.pitch_angle += imu.gyro[0] * dt;
        }
    }

    fn desired(&self) -> (i32, i32) {
        let dx = self.yaw_angle * SENSITIVITY_X;
        let dy = self.pitch_angle * SENSITIVITY_Y;
        (self.start_x + dx.round() as i32, self.start_y + dy.round() as i32)
    }
}

fn spawn_imu_thread() -> Receiver<XOImu> {
    let (tx, rx) = mpsc::channel::<XOImu>();
    thread::Builder::new()
        .name("imu-reader".into())
        .spawn(move || {
            let mut xreal =
                XrealOne::new().expect("failed to connect to XReal One glasses");
            loop {
                match xreal.next() {
                    Ok(imu) => {
                        if tx.send(imu).is_err() {
                            // main thread dropped the receiver — we're done.
                            return;
                        }
                    }
                    Err(e) => {
                        eprintln!("IMU read error: {e:?}");
                        return;
                    }
                }
            }
        })
        .expect("failed to spawn IMU thread");
    rx
}

#[cfg(windows)]
fn spawn_kb_hook_thread() {
    thread::Builder::new()
        .name("kb-hook".into())
        .spawn(|| kb_hook::run())
        .expect("failed to spawn keyboard hook thread");
}

fn main() {
    #[cfg(windows)]
    enable_per_monitor_dpi_awareness();

    let imu_rx = spawn_imu_thread();

    #[cfg(windows)]
    spawn_kb_hook_thread();

    let mut enigo = Enigo::new(&Settings::default()).expect("failed to init enigo");

    println!(
        "ready. hold Right Alt to activate head tracking; release to freeze. Ctrl-C to exit."
    );

    let mut session: Option<Session> = None;
    let mut last_move = Instant::now() - MOUSE_UPDATE_INTERVAL;

    // Debug counters
    let mut samples_since_debug: u32 = 0;
    let mut last_gyro = [0.0f32; 3];
    let mut last_debug = Instant::now();
    let mut last_active_state = false;

    loop {
        let active = RIGHT_ALT_DOWN.load(Ordering::Acquire);

        // State transitions.
        if active && !last_active_state {
            // Just activated — start a fresh session. This recaptures the
            // cursor origin and zeroes accumulated angles, so any drift
            // from a previous session is erased.
            session = Some(Session::new(&enigo));
            if let Some(s) = &session {
                println!(
                    "[ACTIVE] session started at ({}, {})",
                    s.start_x, s.start_y
                );
            }
        } else if !active && last_active_state {
            session = None;
            println!("[IDLE]");
        }
        last_active_state = active;

        // Drain IMU samples that have queued up since last tick.
        loop {
            match imu_rx.try_recv() {
                Ok(imu) => {
                    samples_since_debug += 1;
                    last_gyro = imu.gyro;
                    if let Some(s) = session.as_mut() {
                        s.integrate(&imu);
                    }
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    panic!("IMU thread died");
                }
            }
        }

        let now = Instant::now();

        // Only drive the mouse when active and our throttle allows it.
        if active && now.duration_since(last_move) >= MOUSE_UPDATE_INTERVAL {
            last_move = now;
            if let Some(s) = session.as_ref() {
                let (desired_x, desired_y) = s.desired();
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
        }

        if now.duration_since(last_debug) >= DEBUG_INTERVAL {
            last_debug = now;
            let state = if active { "ACTIVE" } else { "IDLE  " };
            match session.as_ref() {
                Some(s) => {
                    let (dx, dy) = s.desired();
                    let (cx, cy) = enigo.location().unwrap_or((-999, -999));
                    println!(
                        "[{state}] samples={:>4}  gyro=[{:>7.4},{:>7.4},{:>7.4}]  yaw={:>7.3} pitch={:>7.3}  desired=({:>5},{:>5})  actual=({:>5},{:>5})",
                        samples_since_debug,
                        last_gyro[0], last_gyro[1], last_gyro[2],
                        s.yaw_angle, s.pitch_angle, dx, dy, cx, cy,
                    );
                }
                None => {
                    println!(
                        "[{state}] samples={:>4}  gyro=[{:>7.4},{:>7.4},{:>7.4}]",
                        samples_since_debug, last_gyro[0], last_gyro[1], last_gyro[2],
                    );
                }
            }
            samples_since_debug = 0;
        }

        // Small sleep so we don't burn a core. The 8ms mouse throttle and
        // mpsc draining mean we don't need to spin faster than ~1ms.
        thread::sleep(Duration::from_micros(500));
    }
}
