%% phaseC_selftest.m - checks the Phase C safety logic with made-up positions.
% Uses the default site in uwb_config.m (Lane 1 y = 3.4 left->right,
% Lane 2 y = 2.0 right->left, x = 1.5 ... 10.5). All lines should say PASS.
clear; clc;
cfg = uwb_config();
T = cell(0, 2);
dt = 0.04; sig = 0.15;
mk = @(p, v) struct('pos', p, 'vel', v, 'sigmaPos', sig, 'stale', false, 'speed', norm(v));
NORMAL = 1; SLOW = 2; CRAWL = 3; STOP = 4;
idx = @(S, name) find(strcmp({S.agv.name}, name));

%% 1. Person outside the fence, standing still -> every AGV NORMAL
S = safety_init(cfg);
for ts = 0:dt:3, [S, o] = safety_step(S, ts, mk([6 -2], [0 0]), cfg); end
T(end+1, :) = {'outside fence: all AGVs NORMAL', all(o.applied == NORMAL)};

%% 2. Person standing in Lane 1 -> only the AGV still heading to them stops (E2)
S = safety_init(cfg);
S.agv(idx(S, 'L1-A')).s = 1.0;      % x = 2.5, person ahead of it
S.agv(idx(S, 'L1-B')).s = 6.0;      % x = 7.5, already 1.5 m past the person
for ts = 0:dt:1.0, [S, o] = safety_step(S, ts, mk([6 3.4], [0 0]), cfg); end
T(end+1, :) = {'in Lane 1: approaching AGV L1-A STOPs', o.applied(idx(S, 'L1-A')) == STOP};
T(end+1, :) = {'in Lane 1: AGV L1-B that has passed stays NORMAL', o.applied(idx(S, 'L1-B')) == NORMAL};
T(end+1, :) = {'in Lane 1: Lane 2 (1.4 m away) only SLOW for approaching AGV', ...
    o.applied(idx(S, 'L2-A')) == SLOW && o.applied(idx(S, 'L2-B')) == NORMAL};

%% 3. Braking: L1-A stands still within latency + vMax/decel
tLimit = cfg.agv.latency + cfg.agv.vMax / cfg.agv.decel + 0.1;
for ts = 1.04:dt:tLimit, [S, o] = safety_step(S, ts, mk([6 3.4], [0 0]), cfg); end
T(end+1, :) = {sprintf('braking: L1-A at 0 m/s within %.1f s', tLimit), o.v(idx(S, 'L1-A')) == 0};

%% 4. Command latency: level changes reach the AGV after cfg.agv.latency
S = safety_init(cfg); S.agv(idx(S, 'L1-A')).s = 1.0;
for ts = 0:dt:1, [S, o] = safety_step(S, ts, mk([6 -2], [0 0]), cfg); end      % all NORMAL
t0 = 1.04; [S, o] = safety_step(S, t0, mk([6 3.4], [0 0]), cfg);              % step into the lane
early = o.applied(idx(S, 'L1-A'));
for ts = t0 + dt:dt:t0 + cfg.agv.latency + dt, [S, o] = safety_step(S, ts, mk([6 3.4], [0 0]), cfg); end
T(end+1, :) = {sprintf('latency: STOP applied after %.1f s, not before', cfg.agv.latency), ...
    early == NORMAL && o.applied(idx(S, 'L1-A')) == STOP};

%% 5. Fast approach triggers STOP further from the lane than a slow one (E1)
dStop = nan(1, 2); speeds = [1.2 0.3];
for c = 1:2
    S = safety_init(cfg); S.agv(idx(S, 'L1-A')).s = 1.0;
    p = [6 3.7 + 1.7];  v = [0 -speeds(c)];                  % 1.7 m above Lane 1 edge
    for ts = 0:dt:10
        p = p + v * dt;
        [S, o] = safety_step(S, ts, mk(p, v), cfg);
        if o.lane(1).level == STOP, dStop(c) = o.lane(1).d; break; end
    end
end
T(end+1, :) = {sprintf('E1: fast walker STOPs at %.1f m, slow at %.1f m from lane', dStop), dStop(1) > dStop(2) + 0.5};

%% 6. Data lost -> fail-safe CRAWL (E4)
S = safety_init(cfg);
for ts = 0:dt:2, [S, o] = safety_step(S, ts, mk([6 -2], [0 0]), cfg); end
for ts = 2.04:dt:3, [S, o] = safety_step(S, ts, [], cfg); end                   % no frames for 1 s
T(end+1, :) = {'E4: no data for 1 s -> all AGVs at least CRAWL', o.stale && all(o.applied >= CRAWL)};

%% 7. Data lost while someone is in the lane -> STOP is kept (E4)
S = safety_init(cfg); S.agv(idx(S, 'L1-A')).s = 1.0;
for ts = 0:dt:1, [S, o] = safety_step(S, ts, mk([6 3.4], [0 0]), cfg); end
for ts = 1.04:dt:3, [S, o] = safety_step(S, ts, [], cfg); end
T(end+1, :) = {'E4: data lost during a STOP -> STOP is kept', o.applied(idx(S, 'L1-A')) == STOP};

%% 8. High uncertainty is treated as stale (E4)
S = safety_init(cfg); e = mk([6 -2], [0 0]); e.sigmaPos = 0.8;
for ts = 0:dt:1, [S, o] = safety_step(S, ts, e, cfg); end
T(end+1, :) = {'E4: sigmaPos 0.8 m -> treated as stale', o.stale && all(o.applied >= CRAWL)};

%% 9. Bigger uncertainty -> bigger buffer (E3a)
S = safety_init(cfg); [~, o1] = safety_step(S, 0, mk([6 5], [0 0]), cfg);
e = mk([6 5], [0 0]); e.sigmaPos = 0.3; S = safety_init(cfg); [~, o2] = safety_step(S, 0, e, cfg);
T(end+1, :) = {sprintf('E3a: buffer %.2f m (sigma 0.15) < %.2f m (sigma 0.30)', o1.B, o2.B), o2.B > o1.B};

%% 10. No flicker when standing on the SLOW boundary with +-0.2 m jitter (E5)
S = safety_init(cfg); B = cfg.risk.bufBase + cfg.risk.bufK * sig;
yB = 3.4 + cfg.site.laneWidth / 2 + B + cfg.risk.slowBand;   % exactly on the SLOW boundary
rel = 0; prev = NORMAL;
for ts = 0:dt:30
    [S, o] = safety_step(S, ts, mk([6 yB + 0.2 * sin(2*pi*1.5*ts)], [0 0]), cfg);
    rel = rel + (prev == SLOW && o.lane(1).level == NORMAL); prev = o.lane(1).level;
end
T(end+1, :) = {'E5: no flicker on a boundary in 30 s', rel == 0};

%% 11. Just outside the fence near a lane end -> NORMAL (spec: outside = normal)
S = safety_init(cfg);
for ts = 0:dt:2, [S, o] = safety_step(S, ts, mk([0.2 3.4], [0 0]), cfg); end
T(end+1, :) = {'outside fence next to a lane end: NORMAL', all(o.applied == NORMAL) && ~o.fenceOn};

%% 12. An AGV catching up with a stopped AGV queues behind it (no collision)
% L1-A is stopped just before the person (x = 3.5). L1-B has passed the person
% (so it is NORMAL), drives to the lane end, loops back and catches up with L1-A.
S = safety_init(cfg); L = S.lane(1).L;
S.agv(idx(S, 'L1-A')).s = 1.0;
S.agv(idx(S, 'L1-B')).s = 7.5;
gapMin = Inf;
for ts = 0:dt:8
    [S, o] = safety_step(S, ts, mk([3.5 3.4], [0 0]), cfg);
    gapMin = min(gapMin, mod(S.agv(idx(S, 'L1-A')).s - S.agv(idx(S, 'L1-B')).s, L) - cfg.agv.length);
end
T(end+1, :) = {sprintf('AGVs keep >= %.1f m gap (closest %.2f m)', cfg.agv.minGap, gapMin), ...
    gapMin >= cfg.agv.minGap - 0.05 && o.v(idx(S, 'L1-B')) == 0};

for k = 1:size(T, 1)
    if T{k, 2}, res = 'PASS'; else, res = '*** FAIL ***'; end
    fprintf('%-62s %s\n', T{k, 1}, res);
end
fprintf('\n%d PASS, %d FAIL\n', sum([T{:, 2}]), sum(~[T{:, 2}]));
