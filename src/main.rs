// armouse: head-motion mouse control for XReal One glasses on Windows.
//
// Controls:
//   - Shift+CapsLock: toggle mouse mode on/off (sticky). The Caps Lock
//     state is NOT affected — we swallow both the Shift and Caps Lock
//     events in this chord so Windows never sees it.
//   - Left Ctrl + CapsLock (hold): enable mouse mode while held, disable
//     on release. The CapsLock event is swallowed; Left Ctrl passes
//     through on release so no modifier ever gets stuck. If sticky mode
//     was already on, the hold is a no-op (it won't clobber sticky state).
//   - While mouse mode is active:
//       * head motion drives the cursor
//       * Right Alt  = left click  (press/release, so drag works)
//       * Right Ctrl = right click (press/release)
//     Right Ctrl and Right Alt events are consumed while active. When
//     inactive they pass through untouched — so Left Ctrl/Alt shortcuts
//     (Ctrl+C, Alt+Tab, etc.) are never affected, even in active mode.
//
// Threading model:
//   - IMU reader thread: loops xreal.next() and pushes samples through an
//     mpsc channel. Keeps the TCP socket drained even while idle.
//   - Keyboard hook thread: installs a WH_KEYBOARD_LL hook and runs a
//     Windows message pump. Observes shift state and the Shift+CapsLock
//     chord; flips MOUSE_MODE_ACTIVE; routes click events to main.
//   - Main thread: owns Enigo. Each tick it drains both channels, integrates
//     gyro into the current Session, nudges the cursor toward desired, and
//     emits left/right mouse press/release events.
//
// Axis mapping (empirical):
//   gyro[0] = pitch (negative = looking down)
//   gyro[1] = roll  (unused)
//   gyro[2] = yaw   (negative = looking left)

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};
use std::sync::OnceLock;
use std::thread;
use std::time::{Duration, Instant};

use enigo::{Button, Coordinate, Direction, Enigo, Mouse, Settings};
use xreal_one_driver::{XOImu, XrealOne};

#[derive(Debug, Clone, Copy)]
enum ClickEvent {
    Left(Direction),
    Right(Direction),
}

// Channel the keyboard hook uses to send click events to main. Wrapped in a
// OnceLock because the hook callback is a plain extern fn (no closure capture).
static CLICK_TX: OnceLock<Sender<ClickEvent>> = OnceLock::new();

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
//   MOUSE_MODE_ACTIVE: toggled by Shift+CapsLock. When true, head motion
//   drives the cursor and Left Ctrl / Left Alt become click chords.
static MOUSE_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

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
    use super::{ClickEvent, CLICK_TX, MOUSE_MODE_ACTIVE};
    use enigo::Direction;
    use std::sync::atomic::{AtomicBool, Ordering};
    use windows::Win32::Foundation::{LPARAM, LRESULT, WPARAM};
    use windows::Win32::UI::Input::KeyboardAndMouse::{
        VK_CAPITAL, VK_LCONTROL, VK_LSHIFT, VK_RCONTROL, VK_RMENU, VK_RSHIFT,
    };
    use windows::Win32::UI::WindowsAndMessaging::{
        CallNextHookEx, DispatchMessageW, GetMessageW, SetWindowsHookExW, TranslateMessage,
        HHOOK, KBDLLHOOKSTRUCT, MSG, WH_KEYBOARD_LL, WM_KEYDOWN, WM_KEYUP, WM_SYSKEYDOWN,
        WM_SYSKEYUP,
    };

    // Local modifier tracking. The hook sees every key event so we can
    // trust these; we don't need to query Windows.
    static LSHIFT_DOWN: AtomicBool = AtomicBool::new(false);
    static RSHIFT_DOWN: AtomicBool = AtomicBool::new(false);
    static LCTRL_DOWN: AtomicBool = AtomicBool::new(false);

    // True while the user is holding Left Ctrl + Caps Lock to force mouse
    // mode on. On entering hold mode we remember whether sticky mode was
    // already active, so releasing the hold restores the prior state
    // instead of blindly turning mouse mode off.
    static HOLD_ACTIVE: AtomicBool = AtomicBool::new(false);
    static HOLD_PREV_STICKY: AtomicBool = AtomicBool::new(false);

    fn shift_held() -> bool {
        LSHIFT_DOWN.load(Ordering::Acquire) || RSHIFT_DOWN.load(Ordering::Acquire)
    }

    fn exit_hold_mode() {
        if HOLD_ACTIVE.swap(false, Ordering::AcqRel) {
            // Restore mouse mode to whatever sticky state was before the
            // hold began.
            let prev = HOLD_PREV_STICKY.load(Ordering::Acquire);
            MOUSE_MODE_ACTIVE.store(prev, Ordering::Release);
        }
    }

    unsafe extern "system" fn low_level_keyboard_proc(
        code: i32,
        wparam: WPARAM,
        lparam: LPARAM,
    ) -> LRESULT {
        if code < 0 {
            return CallNextHookEx(HHOOK::default(), code, wparam, lparam);
        }

        let kbd = &*(lparam.0 as *const KBDLLHOOKSTRUCT);
        let msg = wparam.0 as u32;
        let is_down = matches!(msg, WM_KEYDOWN | WM_SYSKEYDOWN);
        let is_up = matches!(msg, WM_KEYUP | WM_SYSKEYUP);
        let vk = kbd.vkCode;

        // Shift: track state, always pass through.
        if vk == VK_LSHIFT.0 as u32 {
            if is_down {
                LSHIFT_DOWN.store(true, Ordering::Release);
            } else if is_up {
                LSHIFT_DOWN.store(false, Ordering::Release);
            }
            return CallNextHookEx(HHOOK::default(), code, wparam, lparam);
        }
        if vk == VK_RSHIFT.0 as u32 {
            if is_down {
                RSHIFT_DOWN.store(true, Ordering::Release);
            } else if is_up {
                RSHIFT_DOWN.store(false, Ordering::Release);
            }
            return CallNextHookEx(HHOOK::default(), code, wparam, lparam);
        }

        // Left Ctrl: track state, always pass through. If we're in hold
        // mode and LCtrl comes up, exit hold mode (but let the key event
        // itself propagate so Windows sees the modifier release cleanly).
        if vk == VK_LCONTROL.0 as u32 {
            if is_down {
                LCTRL_DOWN.store(true, Ordering::Release);
            } else if is_up {
                LCTRL_DOWN.store(false, Ordering::Release);
                exit_hold_mode();
            }
            return CallNextHookEx(HHOOK::default(), code, wparam, lparam);
        }

        // Caps Lock: chords take precedence over the normal behaviour.
        if vk == VK_CAPITAL.0 as u32 {
            // Left Ctrl + CapsLock: hold-to-track. Engaging on key-down,
            // disengaging on key-up. Swallow both so Caps Lock state is
            // untouched.
            if LCTRL_DOWN.load(Ordering::Acquire) {
                if is_down {
                    // Only arm hold-mode on the first down edge — auto-
                    // repeats will hit this path too but the swap below
                    // makes them idempotent.
                    if !HOLD_ACTIVE.swap(true, Ordering::AcqRel) {
                        HOLD_PREV_STICKY
                            .store(MOUSE_MODE_ACTIVE.load(Ordering::Acquire), Ordering::Release);
                        MOUSE_MODE_ACTIVE.store(true, Ordering::Release);
                    }
                } else if is_up {
                    exit_hold_mode();
                }
                return LRESULT(1);
            }

            if shift_held() {
                if is_down {
                    // Toggle on the key-down edge, ignore auto-repeats by
                    // simply flipping (a held Caps Lock won't flood because
                    // the OS sends one WM_KEYDOWN, not a stream).
                    let prev = MOUSE_MODE_ACTIVE.fetch_xor(true, Ordering::AcqRel);
                    let _ = prev; // value before toggle — unused
                }
                return LRESULT(1);
            }

            // No modifier chord — let Caps Lock work normally.
            return CallNextHookEx(HHOOK::default(), code, wparam, lparam);
        }

        // Right Ctrl / Right Alt: intercept only while mouse mode is active.
        let click_key = vk == VK_RCONTROL.0 as u32 || vk == VK_RMENU.0 as u32;
        if click_key && MOUSE_MODE_ACTIVE.load(Ordering::Acquire) && (is_down || is_up) {
            let dir = if is_down {
                Direction::Press
            } else {
                Direction::Release
            };
            let event = if vk == VK_RMENU.0 as u32 {
                ClickEvent::Left(dir)
            } else {
                ClickEvent::Right(dir)
            };
            if let Some(tx) = CLICK_TX.get() {
                let _ = tx.send(event);
            }
            // Swallow so Windows doesn't see a bare Alt-up (which would
            // open the window menu bar) or fire Ctrl-based shortcuts.
            return LRESULT(1);
        }

        CallNextHookEx(HHOOK::default(), code, wparam, lparam)
    }

    /// Installs the hook and runs a message pump. Never returns.
    pub fn run() -> ! {
        unsafe {
            let _hook = SetWindowsHookExW(WH_KEYBOARD_LL, Some(low_level_keyboard_proc), None, 0)
                .expect("failed to install WH_KEYBOARD_LL hook");
            let mut msg = MSG::default();
            while GetMessageW(&mut msg, None, 0, 0).as_bool() {
                let _ = TranslateMessage(&msg);
                DispatchMessageW(&msg);
            }
        }
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

    // Click-event channel must be installed BEFORE the hook thread starts,
    // otherwise early events would be dropped.
    let (click_tx, click_rx) = mpsc::channel::<ClickEvent>();
    CLICK_TX
        .set(click_tx)
        .expect("CLICK_TX already initialised");

    #[cfg(windows)]
    spawn_kb_hook_thread();

    let mut enigo = Enigo::new(&Settings::default()).expect("failed to init enigo");

    println!(
        "ready. Shift+CapsLock = toggle mouse mode, LCtrl+CapsLock = hold mouse mode. Right Alt = left click, Right Ctrl = right click. Ctrl-C to exit."
    );

    let mut session: Option<Session> = None;
    let mut last_move = Instant::now() - MOUSE_UPDATE_INTERVAL;

    // Button-held state — so we can release anything still down when mouse
    // mode deactivates. Without this, toggling off while Right Ctrl is
    // still physically held would leave the left mouse button stuck down.
    let mut left_down = false;
    let mut right_down = false;

    // Debug counters
    let mut samples_since_debug: u32 = 0;
    let mut last_gyro = [0.0f32; 3];
    let mut last_debug = Instant::now();
    let mut last_active_state = false;

    loop {
        let active = MOUSE_MODE_ACTIVE.load(Ordering::Acquire);

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
            // Release any mouse buttons still physically "held" via Ctrl/Shift.
            if left_down {
                enigo
                    .button(Button::Left, Direction::Release)
                    .expect("failed to release left button");
                left_down = false;
            }
            if right_down {
                enigo
                    .button(Button::Right, Direction::Release)
                    .expect("failed to release right button");
                right_down = false;
            }
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

        // Drain click events.
        loop {
            match click_rx.try_recv() {
                Ok(ClickEvent::Left(dir)) => {
                    enigo
                        .button(Button::Left, dir)
                        .expect("failed to send left click");
                    left_down = matches!(dir, Direction::Press);
                }
                Ok(ClickEvent::Right(dir)) => {
                    enigo
                        .button(Button::Right, dir)
                        .expect("failed to send right click");
                    right_down = matches!(dir, Direction::Press);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => break,
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
