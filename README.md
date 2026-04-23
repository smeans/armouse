# armouse

Turn your head, move the mouse. A Windows proof-of-concept that reads the IMU
from XReal One AR glasses over TCP and drives the OS mouse cursor from head
motion.

> Status: **early POC**. Always-on head-to-cursor tracking works. Hotkey gating
> and click bindings are not implemented yet — see [Roadmap](#roadmap).

## How it works

- [`xreal_one_driver`](https://github.com/rohitsangwan01/xreal_one_driver)
  connects to the glasses at `169.254.2.1:52998` and streams IMU samples at
  ~1 kHz (gyro + accel + microsecond timestamp).
- Each sample integrates yaw (gyro Z) and pitch (gyro X) into accumulated
  angles since the session started.
- Every 8 ms (~125 Hz), the program computes a desired cursor position
  `start + angle * sensitivity` and nudges the cursor toward it using a
  relative `SendInput` delta via [`enigo`](https://crates.io/crates/enigo).
- The process declares itself **per-monitor DPI-aware v2** at startup so
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

On launch, the current cursor position is captured as the session origin.
Look around and the cursor will track your head. `Ctrl-C` to quit.

The release profile matters — debug builds drop IMU samples under load.

## Configuration

Tweakable constants at the top of [`src/main.rs`](src/main.rs):

| constant                  | default | meaning                              |
| ------------------------- | ------- | ------------------------------------ |
| `SENSITIVITY_X`           | `500.0` | pixels per radian of yaw             |
| `SENSITIVITY_Y`           | `500.0` | pixels per radian of pitch           |
| `MOUSE_UPDATE_INTERVAL`   | `8 ms`  | how often we inject mouse moves      |
| `DEBUG_INTERVAL`          | `200 ms`| debug print rate                     |

Flip the sign of either sensitivity constant if your setup moves the wrong
way.

## Roadmap

- [x] **Milestone 1** — Cargo setup, read 10 IMU samples.
- [x] **Milestone 2** — Identify pitch/yaw/roll axes empirically.
- [x] **Milestone 3** — Always-on head-to-cursor tracking across multi-monitor
      DPI-scaled setups.
- [ ] **Milestone 4** — Hold Right Alt to activate mouse mode; release to
      freeze cursor. Naturally resets session drift on each press.
- [ ] **Milestone 5** — Right Ctrl = left click, Right Shift = right click
      (press/release semantics for drag support).

Known trade-off for milestone 4: `global-hotkey` wraps Windows `RegisterHotKey`
which **swallows** the key, so Right Alt will not pass through to other apps
while armouse is running (this breaks AltGr for international keyboards).

## License

No license yet. All rights reserved until one is added.
