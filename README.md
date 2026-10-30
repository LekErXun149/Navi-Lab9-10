# UWB AGV safety - MATLAB code

Tag (USB) -> decode Node_Frame2 -> height-corrected trilateration -> Kalman filter
(position, velocity, speed, heading, uncertainty) -> AGV safety logic -> simulated AGVs.

- **Phase A:** localisation pipeline (decode, trilateration, Kalman filter, live/replay)
- **Phase B:** risk thresholds derived from recorded data (`phaseB_tuning.m`)
- **Phase C:** AGV safety logic, simulated AGVs and display (`phaseC_run.m`)

Requires MATLAB R2019b+ (uses `serialport`). No toolboxes needed.
Also runs in GNU Octave 8 for everything except live serial.

## Files

| File | What it does |
|---|---|
| `uwb_config.m` | **All settings.** Anchors, tag height, COM port, filter tuning, site layout (`cfg.site`), AGV numbers (`cfg.agv`), risk thresholds (`cfg.risk`), fail-safe (`cfg.health`) |
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
| `safety_init.m` | Initial state: lanes, AGVs spread along each lane, warning memories |
| `safety_step.m` | **The safety logic**, one step: buffer (E3a), time-to-lane (E1), STOP/SLOW per lane with hysteresis (E5), only AGVs heading to the person respond (E2), fail-safe (E4), command latency, AGV motion with no-collision queueing |
| `safety_run_offline.m` | Runs a recording through localisation + safety logic on a fixed time step (no wall clock) - used by `phaseC_offline` and Phase D |
| `safety_draw.m` | Phase C display: fence, lanes and AGVs coloured by level, person with buffer, status panel |
| `phaseC_run.m` | **Main program:** live tag (or replay) -> safety logic -> simulated AGVs, with display and log |
| `phaseC_offline.m` | Runs the safety system on the recordings; time per level, STOP events, closest distance, timeline figures |
| `phaseC_selftest.m` | 15 checks of the safety logic with made-up positions |
| `geo_seg.m`, `geo_poly_dist.m` | Geometry: distance to a lane segment, inside/distance to the fence |
| `uwb_prctile.m`, `uwb_nansum.m`, `uwb_nanmean.m`, `ternary_str.m` | Small helpers (no Statistics Toolbox needed) |
| `data/` | The four 26-Oct recordings |
| `test_fake_serial/` | Fake `serialport` that streams walk 1 - tests live mode without the tag |

## Testing the code

Set MATLAB's **Current Folder** to `phaseA`. Type all commands in the **Command Window**.
Run the tests in order. Repeat them after any code change and before each new anchor setup.
Record results in `TEST_STATUS.md`, not here.

| Group | Tests | Needs |
|---|---|---|
| Phase A | 0 self-test, 1 offline, 2 replay, 3 fake port | Nothing |
| Phase A | 4 real tag, no anchors | Tag + USB cable |
| Phase A | 5 full system | Tag + 4 anchors |
| Phase B | B-1 to B-4 tuning | Nothing |
| Phase C | C-1 to C-4 | Nothing |
| Phase C | C-5 real tag, no anchors | Tag + USB cable |
| Phase C | C-6 full system | Tag + 4 anchors |

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

### Phase B tests

**B-1 Default tuning run.** `phaseB_tuning`
**Pass:** about 2 minutes, four evidence figures, ends with `uwb_config.m is up to date`,
`TUNING_RESULTS.md` updated.

**B-2 Values.** Open `TUNING_RESULTS.md`. **Pass:** vApproach 0.50, tStop 3.0, tSlow 4.0,
buffer 0.4 m + 2 sigma, cap 1.25 m, gap 0.45 m, dwell 1.0 s.

**B-3 Re-tune with chosen files, including a raw `.bin`** (the route used after a new anchor setup).
Find a `.bin` from test 3 with `dir logs/*.bin` (repeat test 3 if there is none), then:
```matlab
files    = {'data/underA3_1620.xlsx', 'logs/raw_<time>.bin'};
isStatic = [true false];
truthXY  = [12.093 0; NaN NaN];
phaseB_tuning
```
**Pass:** runs without errors and prints the recommendation table. `<-- update` marks are
expected here - do not copy these values into `uwb_config.m`.

**B-4 Back to the defaults.**
```matlab
clear files isStatic truthXY
phaseB_tuning
```
**Pass:** ends with `uwb_config.m is up to date`; `TUNING_RESULTS.md` lists the four 26-Oct recordings.

### Phase C tests

**C-1 Safety logic self-test.**
```matlab
phaseC_selftest
```
**Pass:** `15 PASS, 0 FAIL`. Made-up positions check: outside the fence -> NORMAL; person in
Lane 1 -> only the AGV heading to them STOPs, the one that has passed stays NORMAL, Lane 2
only SLOW (E2); braking time; command latency; a fast walker triggers STOP further from the
lane than a slow one (E1); data lost -> CRAWL, a STOP is kept, high uncertainty counts as
lost (E4); larger uncertainty -> larger buffer (E3a); no flicker on a boundary (E5);
outside the fence next to a lane end -> NORMAL; an AGV catching up with a slower one
queues behind it; an AGV coming round the loop stops on the feeder track before a person
standing at the lane start.

**C-2 Offline run on the recordings.**
```matlab
phaseC_offline
```
**Pass:** three figures (map + level timeline per recording), no errors. Printed table close to:

| Recording | Check |
|---|---|
| `walk1_1704` | fence INSIDE ~64%; L1-A STOP ~34%; closest on-lane AGV to the person >= ~1.2 m |
| `walk2` | fence INSIDE ~36%; every AGV NORMAL 71-84% |
| `static_1533` | person stands ~1.4 m from the Lane 2 start: Lane 1 AGVs SLOW ~99%, Lane 2 AGVs mostly NORMAL/SLOW |

**C-3 Replay with display.** In `phaseC_run.m` set `MODE = "replay";`, then `phaseC_run`.

| Check | Pass if |
|---|---|
| Layout | Anchors, red dashed fence, two lanes with direction labels at the exit end, dotted feeder track before each lane start, four AGVs |
| Colours | Lanes and AGVs change green / amber / red as the person gets near |
| Direction rule (E2) | When the person is at a lane, the AGV that has already passed stays green and keeps moving |
| No collisions | AGVs never overlap; a following AGV shows `queued` and waits |
| Re-entry | An AGV coming round (faded, dashed outline on the feeder track) stops there if the person is near the lane start |
| Panel | Distance, TTL and reason per lane; level and speed per AGV |
| End | Closing the window prints the time per level and saves `logs/phaseC_<time>.mat` |

**C-4 Live path with the fake serial port.** Set `MODE = "serial";`, then:
```matlab
addpath('test_fake_serial')
phaseC_run
rmpath('test_fake_serial')
clear classes
```
**Pass:** walk 1 plays at ~20x, the display behaves as in C-3, no errors. Then close the
window. `which serialport` must point into MATLAB's toolbox folder again.

**C-5 Real tag, no anchors - fail-safe (E4).** Plug in the tag, close NAssistant, `MODE = "serial";`, `phaseC_run`.

| Check | Pass if |
|---|---|
| No position | Panel shows `DATA STALE -> fail-safe`, every AGV slows to CRAWL (0.2 m/s), orange |
| Unplug USB | Still CRAWL, MATLAB does not crash |
| Port release | Close the window, run again - reconnects without "port in use" |

**C-6 Full system - after a new anchor setup.** Update `cfg.site` to the taped fence and
lanes, then walk the live demo script (outside -> parallel -> slow approach -> fast approach
-> into Lane 1 -> stand at the edge -> unplug -> walk out) and check each response.

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

## Running the safety system (Phase C)

```matlab
phaseC_run
```
`MODE = "serial"` for the live tag, `"replay"` for a recording (`REPLAYFILE`, `REPLAYSPEED`).
Close the figure to stop; the log is saved to `logs/phaseC_<time>.mat` and, live, the raw
bytes to `logs/raw_<time>.bin`.

Levels: 1 NORMAL (100%), 2 SLOW (50%), 3 CRAWL (20%), 4 STOP, from `cfg.agv.levels`.

| Rule | Behaviour |
|---|---|
| Fence | Person outside `cfg.site.fence` -> every AGV NORMAL |
| Buffer (E3a) | `B = min(bufBase + bufK * sigmaPos, bufMax)`; residual (E3b) enters through the filter's measurement noise |
| STOP | distance to lane edge `< B`, or time-to-lane `< tStop` |
| SLOW | distance `< B + slowBand`, or time-to-lane `< tSlow` |
| Time-to-lane (E1) | distance / speed towards the lane, only if that speed `> vApproach` |
| Direction (E2) | Only an AGV that has not yet passed the person's crossing point (by more than `passMargin`) takes the lane's level |
| Hysteresis (E5) | Each condition ON at once, OFF only when clear by `hystGap` for `dwell` seconds |
| Fail-safe (E4) | No data for `staleTime`, filter stale, or `sigmaPos > maxSigma` -> every AGV at least CRAWL; a STOP is kept |
| AGVs | Commands arrive after `latency`; brake/accelerate at `decel`/`accel`; keep `minGap` to the AGV in front |
| Re-entry | After the lane end an AGV returns out of sight and re-enters along a virtual feeder track of `feedLen` before the lane start (s < 0), where the rules already apply - so it can stop before entering. The feeder is not a physical path: distances to the person count only AGVs on the lane |

`out.overall` (highest level any AGV acts on) is the value for the Leonardo warning light (Phase E).

## Running live

Close NAssistant, check `cfg.serial.port`, `MODE = "serial"`, run `phaseA_live`.
Close the figure to stop. Data saved to `logs/run_*.mat` and raw bytes to `logs/raw_*.bin`
(load the .bin with `nf2_load` or replay it with `MODE = "replay"`).

## New anchor setup: only edit `uwb_config.m`
- `cfg.anchor.xy` - new coordinates from NAssistant
- `cfg.anchor.h`, `cfg.tag.h` - measured heights
- `cfg.serial.port`
- `cfg.site` - fence corners and lane start/end points, measured on the floor
  (lane ends at least ~1.5 m inside the fence: outside the fence the AGVs do not react)
- `cfg.risk` - after re-tuning (see "Tuning the risk thresholds")
