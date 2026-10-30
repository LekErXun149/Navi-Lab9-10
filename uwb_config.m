function cfg = uwb_config()
%UWB_CONFIG  All settings for the UWB AGV-safety system in one place.
%   cfg = uwb_config()
%
%   AFTER A NEW ANCHOR SETUP: update the ANCHOR section (coordinates + heights)
%   and cfg.serial.port, then run phaseA_live. Nothing else should need changing.

%% ---- Anchors (from NAssistant coordinate table) -------------------------
% Layout recorded 26-Oct (16:50 table). Replace after each new anchor setup.
cfg.anchor.xy = [ 0.000  0.000     % A0
                  0.681  7.003     % A1
                 11.958  5.357     % A2
                 12.093  0.000 ];  % A3
cfg.anchor.h  = [1.30 1.30 1.30 1.30];   % antenna height above floor (m), per anchor
cfg.anchor.n  = size(cfg.anchor.xy, 1);

%% ---- Tag ---------------------------------------------------------------
cfg.tag.h = 0.56;          % tag antenna height above floor (m). Best: hold at ~anchor height
cfg.tag.id = 5;            % for information only

%% ---- Serial (tag wired to laptop) --------------------------------------
cfg.serial.port = "COM7";
cfg.serial.baud = 921600;
cfg.serial.rawLogDir = "logs";   % raw bytes saved here during live runs ("" = off)

%% ---- Trilateration -----------------------------------------------------
cfg.tri.minAnchors  = 3;       % need >= 3 ranges for a 2D fix
cfg.tri.maxIter     = 10;      % Gauss-Newton iterations
cfg.tri.minHoriz    = 0.05;    % floor for horizontal range after height correction (m)

%% ---- Kalman filter (constant-velocity, state [x y vx vy]) --------------
% q and sigma0 tuned on the 26-Oct recordings: speed noise when still ~0.1 m/s
% (95%: 0.2-0.36 m/s), reacts to start/stop in ~0.5-0.7 s, walk speed ~0.7 m/s.
cfg.kf.q         = 0.2;    % white-acceleration spectral density ((m/s^2)^2/Hz). Higher = follows turns faster, noisier speed
cfg.kf.sigma0    = 0.30;   % base measurement std (m) per axis
cfg.kf.cRes      = 1.5;    % E3b: extra measurement std per metre of trilateration residual RMS
cfg.kf.gateChi2  = 13.8;   % innovation gate (chi-square, 2 dof, 99.9%). Larger jumps are rejected
cfg.kf.maxReject = 12;     % after this many rejections in a row, re-initialise (~0.5 s at 25 Hz)
cfg.kf.maxGap    = 1.0;    % s; time gap larger than this -> re-initialise filter
cfg.kf.P0pos     = 1.0;    % initial position variance (m^2)
cfg.kf.P0vel     = 1.0;    % initial velocity variance ((m/s)^2)

%% ---- Site: danger area (fence) and AGV lanes (Phase C) -----------------
% Update after a new anchor setup. Keep the fence ~0.5 m inside the anchors
% (accuracy is worst at the anchor boundary) and the lanes near the middle.
cfg.site.fence = [ 0.5 0.5        % danger-area polygon, corners in order (m)
                   1.1 6.3
                  11.5 4.8
                  11.6 0.5 ];
% Each lane: AGVs drive from p0 to p1, then return out of sight and re-enter at p0
% along a feeder track (cfg.agv.feedLen). Keep lane ends >= bufBase + slowBand
% (~1.5 m) inside the fence: outside the fence the AGVs do not react.
cfg.site.lanes(1).name  = 'Lane 1';  cfg.site.lanes(1).p0 = [2.5 3.4];  cfg.site.lanes(1).p1 = [9.5 3.4];
cfg.site.lanes(2).name  = 'Lane 2';  cfg.site.lanes(2).p0 = [9.5 2.0];  cfg.site.lanes(2).p1 = [2.5 2.0];
cfg.site.laneWidth = 0.6;            % m, width of the lane band (AGV width + margin)

%% ---- Simulated AGVs (Phase C uses the same numbers) -------------------
cfg.agv.vMax     = 1.0;              % normal speed (m/s)
cfg.agv.levels   = [1.0 0.5 0.2 0];  % speed fraction for NORMAL, SLOW, CRAWL, STOP
cfg.agv.names    = {'NORMAL', 'SLOW', 'CRAWL', 'STOP'};
cfg.agv.decel    = 0.5;              % braking (m/s^2)
cfg.agv.accel    = 0.3;              % speeding up again (m/s^2)
cfg.agv.latency  = 0.2;              % command latency MATLAB -> AGV (s), assumed
cfg.agv.perLane  = 2;                % AGVs on each lane
cfg.agv.length   = 0.8;              % m
cfg.agv.width    = 0.5;              % m
cfg.agv.passMargin = 0.7;            % m: an AGV whose REAR is this far past the person's crossing point is "passed" (E2)
cfg.agv.minGap   = 0.5;              % m: bumper-to-bumper gap an AGV keeps to the one in front (no collisions)
cfg.agv.feedLen  = 3.0;              % m: feeder track before each lane start; must exceed latency travel +
                                     %    braking distance (0.2 + 1.0 m at 1 m/s) so a re-entering AGV can stop

%% ---- Risk thresholds (Phase B: derived by phaseB_tuning.m) -------------
% Evidence for each value: TUNING_RESULTS.md. Re-run phaseB_tuning after a new
% anchor setup, or after changing the filter settings or the AGV numbers above.
cfg.risk.vApproach = 0.15;   % m/s: slower approach than this is "not approaching" (distance zones still apply)
cfg.risk.tStop     = 2.5;    % s: time-to-lane below this -> STOP
cfg.risk.tSlow     = 3.5;    % s: time-to-lane below this -> SLOW
cfg.risk.bufBase   = 0.40;   % m: fixed part of the safety buffer (covers bias)
cfg.risk.bufK      = 2.0;    % buffer = bufBase + bufK * sigmaPos   (E3a)
cfg.risk.bufMax    = 1.25;   % m: cap on the buffer
cfg.risk.slowBand  = 1.90;   % m: SLOW  if distance < B + slowBand  (standing-start zones, Phase B B4:
cfg.risk.crawlBand = 1.10;   % m: CRAWL if distance < B + crawlBand  a person standing inside can reach the
cfg.risk.stopBand  = 0.70;   % m: STOP  if distance < B + stopBand   lane before a faster AGV could stop)
cfg.risk.hystGap   = 0.45;   % m: release only when this much further out (E5)
cfg.risk.dwell     = 1.0;    % s: release condition must hold this long (E5)
cfg.risk.entryHold  = 1.5;   % s: after entering the fence, lanes within entryReach STOP (Phase D finding:
cfg.risk.entryReach = 2.0;   % m  the speed estimate lags ~1 s after a turn). = 0.8 m/s walk x tStop (B4)

%% ---- Safety policy (Phase D compares versions; keep 'full' for normal use) --
cfg.policy.mode     = 'full';  % 'full' = this project's system, 'basic' = stop all AGVs inside the fence
cfg.policy.E1       = true;    % time-to-lane prediction
cfg.policy.E2       = true;    % only AGVs heading to the person respond
cfg.policy.E3       = true;    % uncertainty-sized buffer (false -> fixedBuf)
cfg.policy.E5       = true;    % hysteresis + dwell
cfg.policy.fixedBuf = 0.72;    % m, buffer used when E3 is off (= bufBase + bufK x typical sigma 0.16)

%% ---- Health (inputs to the fail-safe in Phase C) -----------------------
cfg.health.staleTime = 0.5;   % s without a good update -> data considered stale
cfg.health.staleLevel = 3;    % E4: while stale, every AGV goes to at least this level (3 = CRAWL)
cfg.health.maxSigma   = 0.5;  % E4: position uncertainty above this (m) is treated as stale, e.g. filter start-up
end
