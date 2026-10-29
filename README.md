# UWB AGV safety - MATLAB code

Tag (USB) -> decode Node_Frame2 -> height-corrected trilateration -> Kalman filter
(position, velocity, speed, heading, uncertainty) -> AGV safety logic (Phase C).

- **Phase A:** localisation pipeline (decode, trilateration, Kalman filter, live/replay)
- **Phase B:** risk thresholds derived from recorded data (`phaseB_tuning.m`)

Requires MATLAB R2019b+ (uses `serialport`). No toolboxes needed.
Also runs in GNU Octave 8 for everything except live serial.

## Files

| File | What it does |
|---|---|
| `uwb_config.m` | **All settings.** Anchors, tag height, COM port, filter tuning, AGV numbers (`cfg.agv`), risk thresholds (`cfg.risk`) |
| `nf2_parse.m` | Decodes raw Node_Frame2 bytes (sync, checksum, split frames) |
| `nf2_load.m` | Loads a recording: NAssistant export (.xlsx/.csv) or raw log (.bin) |
| `nf2_empty.m`, `nf2_concat.m`, `nf2_slice.m` | Helpers for the common frame format |
| `UwbSource.m` | One interface for **live serial** and **replay** (same output, so the replay fallback uses identical code) |
| `uwb_trilat.m` | Height correction + linear LS seed + Gauss-Newton; returns residual (E3b) |
| `uwb_kf_init.m`, `uwb_kf_step.m` | Constant-velocity Kalman filter with outlier gate, re-start, uncertainty (E3a), data age |
| `uwb_process.m` | The full per-frame pipeline, used offline **and** live |
| `phaseA_selftest.m` | 17 checks (decoder vs manual example, trilateration, filter, hysteresis). Run first |
| `phaseA_offline.m` | Runs everything on the 26-Oct recordings; prints the numbers for Phase B |
| `phaseA_live.m` | Live display (or replay) with speed, heading, uncertainty, stale warning; logs everything |
| `risk_hyst.m` | Hysteresis + dwell for one warning flag (E5): ON at once, OFF only after the release condition holds for the dwell time |
| `phaseB_tuning.m` | Derives every `cfg.risk` value from recordings, draws the evidence, writes `TUNING_RESULTS.md` |
| `uwb_prctile.m`, `uwb_nansum.m`, `uwb_nanmean.m`, `ternary_str.m` | Small helpers (no Statistics Toolbox needed) |
| `data/` | The four 26-Oct recordings |
| `test_fake_serial/` | Fake `serialport` that streams walk 1 - tests live mode without the tag |

## Testing the code

Set MATLAB's **Current Folder** to `phaseA`. Type all commands in the **Command Window**.
Run the tests in order. Repeat the full set after any code change and before each new anchor setup.

| Step | Needs |
|---|---|
| 0. Self-test | Nothing |
| 1. Offline run | Nothing |
| 2. Replay | Nothing |
| 3. Live path with fake port | Nothing |
| 4. Real tag, no anchors | Tag + USB cable |
| 5. Full system | Tag + 4 anchors |

Record results in `TEST_STATUS.md`, not here.

### 0. Self-test - checks every building block
```matlab
phaseA_selftest
```
**Pass:** `17 PASS, 0 FAIL`. It checks the decoder against the example frame in the
LinkTrack manual (section 8.1.2.5), checksum rejection, trilateration (exact, 3 anchors,
refuses 2 anchors, residual), the Kalman filter (speed, noise reduction, stale data,
outlier gate), and the hysteresis (instant ON, delayed OFF, no flicker). The KF numbers vary slightly between MATLAB and Octave (different random
noise) - only PASS/FAIL matters.

### 1. Offline run - pipeline on the 26-Oct recordings
```matlab
phaseA_offline
```
**Pass:** four figures, no errors. Printed numbers close to:

| Recording | Check |
|---|---|
| `underA3_1620` | RMSE vs tape: NAssistant ~0.95 m, KF ~0.47 m |
| `static_1533` | KF speed 95% ~0.21 m/s (tag still) |
| `walk1_1704`, `walk2` | KF jitter ~3-4 cm vs NAssistant ~24 cm |

### 2. Replay - live display driven by a recording
In `phaseA_live.m` set `MODE = "replay";`, then:
```matlab
phaseA_live
```
**Pass:** the tag moves smoothly around the walk 2 loop, the side panel updates,
closing the window prints `Stopped. ... frames saved to logs/run_*.mat`.

### 3. Live path with the fake serial port - no hardware
In `phaseA_live.m` set `MODE = "serial";`, then:
```matlab
addpath('test_fake_serial')
phaseA_live
```
A warning that `serialport` is shadowed is expected. Walk 1 plays at ~20x speed.
**Pass:** last lines show `1304 frames` and `0 bad checksums`; a `logs/raw_*.bin` file is created.

**Always restore the real serial port afterwards:**
```matlab
rmpath('test_fake_serial')
clear classes
which serialport      % must point into MATLAB's toolbox folder, NOT test_fake_serial
```

### 4. Real tag, no anchors - checks the real COM port and decoder
Plug in the tag, **close NAssistant**, set `cfg.serial.port` in `uwb_config.m`,
keep `MODE = "serial";`, then `phaseA_live`.

| Check | Pass if |
|---|---|
| Connects | No "port busy / not found" error |
| Data arrives | Frame counter rises ~25 per second |
| No anchors | Panel shows `anchors 0` and `data STALE` in red (correct: no fix possible) |
| Unplug USB while running | Panel shows `NO DATA ... STALE`, MATLAB does not crash |
| Port release | Close the window, run again at once - reconnects without "port in use" |
| Raw log | `nf2_load('logs/raw_<time>.bin')` reports frames with 0 bad checksums |

If the counter stays at 0: NAssistant still open, wrong COM number, or tag not at 921600.

### 5. Full system - after a new anchor setup
With anchors running: stand on a taped point and compare the shown position with the
tape; stand still 30 s (speed ~0, data OK); walk a short line (speed ~0.5-1 m/s).

## Tuning the risk thresholds (Phase B)

```matlab
phaseB_tuning
```
Takes about 2 minutes. Prints each step (B1-B7), shows four evidence figures, compares the
recommended values with `cfg.risk` in `uwb_config.m` (marks `<-- update` where they differ),
and writes `TUNING_RESULTS.md` (values + evidence, for the report).

| Step | Measures | Sets |
|---|---|---|
| B1, B2 | Speed noise when still, walking speeds | (evidence) |
| B5 | Error vs tape at a known point | `bufBase`, `bufK`, `bufMax` |
| B3, B4 | Filter reaction time with real noise, AGV braking, false triggers when standing still | `vApproach`, `tStop`, `tSlow` |
| B6 | Flicker when standing on a boundary | `hystGap`, `dwell` |
| B7 | Data gaps while walking | checks `cfg.health.staleTime` |

Fixed inputs, not tuned: `cfg.agv` (assumed AGV braking/latency), `cfg.risk.slowBand`,
and the acceptance limits `maxFalsePerMin`, `maxFlipsPerMin` at the top of `phaseB_tuning.m`.

**Re-tune after a new anchor setup** (anchor layout and tag height change the noise):
1. With `phaseA_live`, record ~60 s standing still on a taped point, then one walk.
   Each run saves `logs/raw_<time>.bin`.
2. Tell the script which files to use, then run it:
```matlab
files    = {'logs/raw_<static>.bin', 'logs/raw_<walk>.bin'};
isStatic = [true false];
truthXY  = [x y; NaN NaN];     % tape position of the static recording
phaseB_tuning
```
3. Copy the recommended values into `cfg.risk` in `uwb_config.m`, run again until it
   prints `uwb_config.m is up to date`. Run `clear files isStatic truthXY` to go back to
   the 26-Oct recordings.

## Running live

Close NAssistant, check `cfg.serial.port`, `MODE = "serial"`, run `phaseA_live`.
Close the figure to stop. Data saved to `logs/run_*.mat` and raw bytes to `logs/raw_*.bin`
(load the .bin with `nf2_load` or replay it with `MODE = "replay"`).

## New anchor setup: only edit `uwb_config.m`
- `cfg.anchor.xy` - new coordinates from NAssistant
- `cfg.anchor.h`, `cfg.tag.h` - measured heights
- `cfg.serial.port`
- `cfg.risk` - after re-tuning (see "Tuning the risk thresholds")
