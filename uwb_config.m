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

%% ---- Simulated AGVs (Phase C uses the same numbers) -------------------
cfg.agv.vMax     = 1.0;              % normal speed (m/s)
cfg.agv.levels   = [1.0 0.5 0.2 0];  % speed fraction for NORMAL, SLOW, CRAWL, STOP
cfg.agv.decel    = 0.5;              % braking (m/s^2)
cfg.agv.accel    = 0.3;              % speeding up again (m/s^2)
cfg.agv.latency  = 0.2;              % command latency MATLAB -> AGV (s), assumed

%% ---- Risk thresholds (Phase B: derived by phaseB_tuning.m) -------------
% Evidence for each value: TUNING_RESULTS.md. Re-run phaseB_tuning after a new
% anchor setup, or after changing the filter settings or the AGV numbers above.
cfg.risk.vApproach = 0.50;   % m/s: slower approach than this is "not approaching" (distance zones still apply)
cfg.risk.tStop     = 3.0;    % s: time-to-lane below this -> STOP
cfg.risk.tSlow     = 4.0;    % s: time-to-lane below this -> SLOW
cfg.risk.bufBase   = 0.40;   % m: fixed part of the safety buffer (covers bias)
cfg.risk.bufK      = 2.0;    % buffer = bufBase + bufK * sigmaPos   (E3a)
cfg.risk.bufMax    = 1.25;   % m: cap on the buffer
cfg.risk.slowBand  = 1.00;   % m: extra distance beyond the buffer that is SLOW (design choice)
cfg.risk.hystGap   = 0.45;   % m: release only when this much further out (E5)
cfg.risk.dwell     = 1.0;    % s: release condition must hold this long (E5)

%% ---- Health (inputs to the fail-safe in Phase C) -----------------------
cfg.health.staleTime = 0.5;   % s without a good update -> data considered stale
end
