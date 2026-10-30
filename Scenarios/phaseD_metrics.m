function m = phaseD_metrics(rec, P, sc, cfg, S)
%PHASED_METRICS  Measure one simulated run against the TRUE path.
%   Safety (true person position vs AGV bodies on the lanes):
%     minClear   closest gap between the person and an AGV faster than CRAWL (m)
%     nearMiss   times an AGV faster than CRAWL speed (0.2 m/s - the speed the
%                system itself treats as acceptable near a person) came within 0.5 m
%     contact    an AGV still moving (> 0.05 m/s) touched the person (gap <= 0)
%     warn       first STOP for a lane -> person enters that lane (s); NaN = never entered
%                or no AGV had to stop
%     stopDist   person's true distance to the lane edge at the first STOP command (m)
%   Productivity:
%     prod       AGV distance driven / distance at full speed (%)
%     stopCmds   number of STOP commands;  stopTime AGV-seconds standing still
%   Behaviour:
%     flicker    lane level changes per lane per minute inside sc.win (if given)
%     stopInWin  a STOP was active at any time inside sc.calmWin (true/false)
%     release    person leaves the lane -> every AGV back to NORMAL (s)
%     failsafe   during each dropout every AGV acts on >= CRAWL (true/false)
%     recover    dropout end -> every AGV NORMAL again (s)
%     held       sc.holdLane stays STOP for the whole sc.holdWin (true/false)
%     kept       AGV sc.keepAgv never commanded STOP inside sc.keepWin (true/false)
%     feeder     AGV sc.feederAgv stays on its feeder track inside sc.feederWin (true/false)

dt = P.t(2) - P.t(1);
use = P.t >= sc.warm;                               % after the warm-up
nA = numel(S.agv); nL = numel(S.lane);
hl = cfg.agv.length / 2; hw = cfg.agv.width / 2; w = cfg.site.laneWidth / 2;

% --- clearance between the person and every AGV body
clr = inf(numel(P.t), nA); touch = false(numel(P.t), 1);
vFast = cfg.agv.vMax * cfg.agv.levels(3);           % CRAWL speed
for k = 1:nA
    Ln = S.lane(S.agv(k).lane);
    rel = P.xy - Ln.a;
    along = rel * Ln.u'; across = rel * Ln.nrm';
    dx = abs(along - rec.s(:, k)) - hl; dy = abs(across) - hw;
    c = hypot(max(dx, 0), max(dy, 0));
    onLane = rec.s(:, k) >= 0;                       % feeder track is virtual
    phys = onLane & rec.v(:, k) > vFast;
    clr(phys, k) = c(phys);
    touch = touch | (onLane & rec.v(:, k) > 0.05 & c <= 0);
end
cmin = min(clr, [], 2); cmin(~use) = Inf;
m.minClear = min(cmin);
close = cmin < 0.5;
m.nearMiss = sum(diff([false; close]) == 1);
m.contact = any(touch & use);

% --- lane entries and warning time
m.warn = NaN; m.stopDist = NaN;
inLane = false(numel(P.t), nL); dEdge = zeros(numel(P.t), nL);
for l = 1:nL
    Ln = S.lane(l); rel = P.xy - Ln.a;
    along = rel * Ln.u'; across = rel * Ln.nrm';
    inLane(:, l) = abs(across) <= w & along >= 0 & along <= Ln.L;
    dEdge(:, l) = abs(across) - w;
end
stopCmd = any(rec.desired == 4, 2);
firstStop = find(use & stopCmd, 1);
if ~isempty(firstStop), m.stopDist = min(dEdge(firstStop, :)); end
for l = 1:nL
    kIn = find(use & inLane(:, l), 1);
    if isempty(kIn), continue; end
    onL = [S.agv.lane] == l;
    st = any(rec.desired(:, onL) == 4, 2);
    k0 = kIn;
    while k0 > 1 && st(k0 - 1) && use(k0 - 1), k0 = k0 - 1; end   % start of the STOP before entry
    if st(kIn), wl = P.t(kIn) - P.t(k0); else, wl = NaN; end
    if isnan(m.warn) || wl < m.warn, m.warn = wl; end
end

% --- productivity
T = sum(use) * dt;
m.prod = 100 * sum(sum(rec.v(use, :))) * dt / (nA * cfg.agv.vMax * T);
m.stopCmds = sum(sum(diff([ones(1, nA); rec.desired(use, :)]) ~= 0 & rec.desired(use, :) == 4));
m.stopTime = sum(sum(rec.v(use, :) < 0.05)) * dt;

% --- behaviour checks
m.flicker = NaN;
if isfield(sc, 'win')
    wv = P.t >= sc.win(1) & P.t <= sc.win(2);
    ch = sum(sum(diff(rec.laneLevel(wv, :)) ~= 0));
    m.flicker = ch / nL / ((sc.win(2) - sc.win(1)) / 60);
end
m.stopInWin = NaN;
if isfield(sc, 'calmWin')
    wv = P.t >= sc.calmWin(1) & P.t <= sc.calmWin(2);
    m.stopInWin = any(any(rec.desired(wv, :) == 4));
end
m.release = NaN;
anyIn = any(inLane, 2) & use;
kOut = find(anyIn, 1, 'last');
if ~isempty(kOut) && kOut < numel(P.t)
    kN = find((1:numel(P.t))' > kOut & all(rec.desired == 1, 2), 1);
    if ~isempty(kN), m.release = P.t(kN) - P.t(kOut); end
end
m.failsafe = true; m.recover = NaN;
if isfield(sc, 'dropout')
    for k = 1:size(sc.dropout, 1)
        wv = P.t >= sc.dropout(k, 1) + cfg.health.staleTime + cfg.agv.latency + dt & P.t < sc.dropout(k, 2);
        m.failsafe = m.failsafe && all(all(rec.applied(wv, :) >= cfg.health.staleLevel));
        kN = find(P.t >= sc.dropout(k, 2) & all(rec.desired == 1, 2), 1);
        if ~isempty(kN), m.recover = P.t(kN) - sc.dropout(k, 2); else, m.recover = Inf; end
    end
end
m.held = NaN;
if isfield(sc, 'holdWin')
    wv = P.t >= sc.holdWin(1) & P.t <= sc.holdWin(2);
    m.held = all(rec.laneLevel(wv, sc.holdLane) == 4);
end
names = {S.agv.name};
m.kept = NaN;
if isfield(sc, 'keepAgv')
    i = find(strcmp(names, sc.keepAgv));
    wv = P.t >= sc.keepWin(1) & P.t <= sc.keepWin(2);
    m.kept = ~any(rec.desired(wv, i) == 4);
end
m.feeder = NaN;
if isfield(sc, 'feederAgv')
    i = find(strcmp(names, sc.feederAgv));
    wv = P.t >= sc.feederWin(1) & P.t <= sc.feederWin(2);
    m.feeder = all(rec.s(wv, i) < 0);
end
end
