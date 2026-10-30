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
| B-1 Default tuning run | PASS | 29-Oct | slowBand / entryReach added 30-Oct: re-run |
| B-2 Values | PASS | 29-Oct | values changed 30-Oct: re-run |
| B-3 Re-tune with chosen files (.bin) | PASS | 29-Oct | |
| B-4 Back to defaults | PASS | 29-Oct | |
| C-1 Safety logic self-test | PASS (18/18) | 30-Oct | Phase D changed the logic: re-run (now 19 checks) |
| C-2 Offline run | PASS | 30-Oct | Phase D changed the logic: re-run |
| C-3 Replay with display | PASS | 30-Oct | Earlier FAIL (re-looping AGV reached the person) fixed by the feeder track |
| C-4 Fake serial port | PASS | 30-Oct | After feeder-track fix |
| C-5 Real tag, no anchors (fail-safe) | PASS | 30-Oct | DATA STALE, all AGVs CRAWL, unplug and reconnect OK |
| C-6 Full system | Not run | | Needs a new anchor setup |
| D-1 Quick run | Not run | | |
| D-2 Full run | Not run | | |
| D-3 Fence clearance study | Not run | | |
| D-4 Viewer | Not run | | |
