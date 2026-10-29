function [E, state] = uwb_process(F, state, cfg)
%UWB_PROCESS  Full Phase A pipeline for a batch of frames (offline AND live).
%   state = [];                          % first call
%   [E, state] = uwb_process(F, state, cfg)
%
%   Per frame: height-corrected trilateration -> E3b measurement noise from
%   residual -> Kalman filter (position, velocity, uncertainty).
%   Output E has one row per input frame:
%     t, fix (raw trilateration [x y]), resRMS, nUsed,
%     pos, vel, speed, heading, sigmaPos (E3a), accepted, reinit,
%     age (s since last good update), stale (true -> fail-safe input)

if isempty(state)
    state.kf   = uwb_kf_init(cfg);
    state.last = [NaN NaN];
end
n = numel(F.t);
E.t        = F.t;
E.fix      = nan(n, 2);  E.resRMS = nan(n, 1);  E.nUsed = zeros(n, 1);
E.pos      = nan(n, 2);  E.vel    = nan(n, 2);
E.speed    = nan(n, 1);  E.heading = nan(n, 1); E.sigmaPos = nan(n, 1);
E.accepted = false(n, 1); E.reinit = false(n, 1);
E.age      = inf(n, 1);  E.stale  = true(n, 1);

for k = 1:n
    [p, info] = uwb_trilat(F.D(k, :), cfg, state.last);
    if info.ok && info.resRMS > 2.0           % seeded solve went wrong: retry from scratch
        [p, info] = uwb_trilat(F.D(k, :), cfg);
    end
    E.nUsed(k) = info.nUsed;
    if info.ok
        E.fix(k, :)  = p;
        E.resRMS(k)  = info.resRMS;
        state.last   = p;
        sigma = hypot(cfg.kf.sigma0, cfg.kf.cRes * info.resRMS);   % E3b
    else
        p = [NaN NaN]; sigma = NaN;
    end
    [state.kf, est] = uwb_kf_step(state.kf, F.t(k), p, sigma);
    E.pos(k, :)   = est.pos;      E.vel(k, :)  = est.vel;
    E.speed(k)    = est.speed;    E.heading(k) = est.heading;
    E.sigmaPos(k) = est.sigmaPos; E.accepted(k) = est.accepted;
    E.reinit(k)   = est.reinit;   E.age(k)     = est.age;
    E.stale(k)    = ~(est.age <= cfg.health.staleTime);
end
end
