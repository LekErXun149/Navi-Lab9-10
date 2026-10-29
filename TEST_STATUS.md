# Phase A - test status

Test procedures are in `README.md` ("Testing the code"). This file records results only.

| Step | Result | Date | Notes |
|---|---|---|---|
| 0. Self-test | PASS (15/15) | 28-Oct | MATLAB. Now 17 checks (hysteresis added 29-Oct): re-run |
| 1. Offline run | PASS | 28-Oct | |
| 2. Replay | PASS | 28-Oct | |
| 3. Live path with fake port | PASS | 28-Oct | 1304 frames, 0 bad checksums |
| 4. Real tag, no anchors | PASS | 29-Oct | Connected, 461 frames, anchors 0, data STALE (expected) |
| 5. Full system | Not run | | Needs a new anchor setup |
