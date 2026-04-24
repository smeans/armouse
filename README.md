# armouse

Turn your head, move the mouse. A Windows proof-of-concept that reads the IMU
from XReal One AR glasses over TCP and drives the OS mouse cursor from head
motion.

> Status: **working POC**. Head-to-cursor tracking, keyboard-gated activation,
> and mouse clicks are all implemented.

## Controls

| Chord                        | Effect                                                         |
| ---------------------------- | -------------------------------------------------------------- |
| `Shift` + `CapsLock`         | Toggle mouse mode on/off (sticky). Caps-lock state unchanged.  |
| `Left Ctrl` + `CapsLock`     | Hold to enable mouse mode; release to disable.                 |
| `Right Alt` (mouse mode on)  | Left mouse button (press / release — drag works).              |
| `Right Ctrl` (mouse mode on) | Right mouse button (press / release).                          |

Notes:

- Both activation chords swallow the `CapsLock` event so the Caps Lock LED /
  text-casing state is never flipped.
- `Left Ctrl` passes through on release, so no modifier ever gets stuck.
- If sticky mode is already on, holding `LCtrl+CapsLock` is a no-op —
  releasing restores the prior sticky state instead of clobbering it.
- `Right Alt` / `Right Ctrl` are only consumed **while mouse mode is active**.
  When inactive they pass through untouched, so `Left Ctrl` / `Left Alt`
  shortcuts (`Ctrl+C`, `Alt+Tab`, etc.) always work normally.
- `Ctrl-C` in the terminal quits.

## How it works

- [`xreal_one_driver`](https://github.com/rohitsangwan01/xreal_one_driver)
  connects to the glasses at `169.254.2.1:52998` and streams IMU samples at
  ~1 kHz (gyro + accel + microsecond timestamp).
- A background thread drains the IMU socket and forwards samples to main over
  an `mpsc` channel.
- A low-level `WH_KEYBOARD_LL` hook thread observes every keystroke, tracks
  modifier state, implements the activation chords, and routes click events
  back to main through a second channel.
- Main thread owns `enigo` (not `Send` on Windows) and each tick:
  1. drains IMU samples, integrating yaw (`gyro[2]`) and pitch (`gyro[0]`)
     into accumulated angles,
  2. drains click events and presses/releases mouse buttons,
  3. every 8 ms (~125 Hz), computes `desired = start + angle * sensitivity`
     and nudges the cursor toward it with a relative `SendInput` delta.
- Process declares itself **per-monitor DPI-aware v2** at startup so
  coordinates and deltas stay in raw physical pixels across mixed-DPI setups
  (primary + secondary + XReal AR virtual display).

### Axis mapping

The driver remaps the raw chip axes to a consistent frame. Empirically:

| gyro component | meaning                                   |
| -------------- | ----------------------------------------- |
| `gyro[0]`      | pitch — negative when looking down        |
| `gyro[1]`      | roll — unused                             |
| `gyro[2]`      | yaw — negative when looking left          |

## Requirements

- Windows 10/11.
- Rust stable (edition 2021). Install via [rustup](https://rustup.rs/).
- XReal One glasses connected and reachable at `169.254.2.1` (the driver's
  hardcoded address — typically a USB-C tethered network interface).

## Build & run

```powershell
cargo run --release
```

The release profile matters — debug builds can drop IMU samples under load.

## Configuration

Tweakable constants at the top of [src/main.rs](src/main.rs):

| constant                | default  | meaning                         |
| ----------------------- | -------- | ------------------------------- |
| `SENSITIVITY_X`         | `-2000`  | pixels per radian of yaw        |
| `SENSITIVITY_Y`         | `2000`   | pixels per radian of pitch      |
| `MOUSE_UPDATE_INTERVAL` | `8 ms`   | how often mouse moves are sent  |
| `DEBUG_INTERVAL`        | `500 ms` | debug print rate                |

Flip the sign of either sensitivity constant if your setup moves the wrong
way.

## Roadmap

- [x] **Milestone 1** — Cargo setup, read 10 IMU samples.
- [x] **Milestone 2** — Identify pitch/yaw/roll axes empirically.
- [x] **Milestone 3** — Always-on head-to-cursor tracking across multi-monitor
      DPI-scaled setups.
- [x] **Milestone 4** — Keyboard-gated activation (sticky toggle + momentary
      hold) via a low-level keyboard hook.
- [x] **Milestone 5** — Left / right click bindings with press/release
      semantics for drag support.

## License

No license yet. All rights reserved until one is added.
