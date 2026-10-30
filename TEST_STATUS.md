# Test status

Test procedures are in `README.md` ("Testing the code"). This file records results only.

| Step | Result | Date | Notes |
|---|---|---|---|
| 0. Self-test | PASS (17/17) | 29-Oct | MATLAB |
| 1. Offline run | PASS | 28-Oct | |
| 2. Replay | PASS | 28-Oct | |
| 3. Live path with fake port | PASS | 28-Oct | 1304 frames, 0 bad checksums |
| 4. Real tag, no anchors | PASS | 29-Oct | Connected, 461 frames, anchors 0, data STALE (expected) |
| 5. Full system | Not run | | Needs a new anchor setup |
| B-1 Default tuning run | PASS | 29-Oct | up to date |
| B-2 Values | PASS | 29-Oct | |
| B-3 Re-tune with chosen files (.bin) | PASS | 29-Oct | |
| B-4 Back to defaults | PASS | 29-Oct | |
| C-1 Safety logic self-test | PASS (14/14) | 30-Oct | Before the feeder-track fix - re-run (now 15 checks) |
| C-2 Offline run | PASS | 30-Oct | Before the feeder-track fix - re-run |
| C-3 Replay with display | FAIL | 30-Oct | Re-looping AGV reached the person at the lane start. Fixed (feeder track) - re-run |
| C-4 Fake serial port | Not run | | |
| C-5 Real tag, no anchors (fail-safe) | Not run | | |
| C-6 Full system | Not run | | Needs a new anchor setup |
