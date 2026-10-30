function EV = safety_events(EV, S, out, cfg)
%SAFETY_EVENTS  Event log for the safety system. Call once per safety_step.
%   EV = safety_events([], S, out, cfg)    first call (creates the log)
%   EV = safety_events(EV, S, out, cfg)    every following step
%   safety_events_write(EV, basename)      writes <basename>_events.csv and _visits.csv
%
%   Events (EV.ev, one row each):
%     FENCE_ENTER / FENCE_EXIT    person crosses the danger-area fence
%     STALE_START / STALE_END     position data lost / back (fail-safe E4)
%     LEVEL_CHANGE                an AGV is commanded a new level, with:
%        latency_s   command -> AGV acting on it
%        settle_s    AGV acting on it -> target speed reached (braking/accelerating)
%        margin_m    for STOP of an AGV that was still approaching: gap from the
%                    stopped AGV's front to the person's crossing point on its lane
%                    (negative = overshoot: it stopped past that point)
%   Visits (EV.vis, one row per stay inside the fence):
%     entry, exit, duration, worst level, STOP commands, closest moving AGV on a
%     lane, warning time = first SLOW/STOP command -> person first inside a lane

ts = out.ts; nA = numel(S.agv);
if isempty(EV)
    EV.ev = cell(0, 15);                            % rows: see row() below
    EV.vis = zeros(0, 7);
    EV.prevDesired = [S.agv.desired];
    EV.prevFence = false; EV.prevStale = true;
    EV.pending = cell(1, nA);                       % open LEVEL_CHANGE per AGV: [row, tCmd, level]
    EV.visit = [];                                  % open visit
    EV.first = true;
    EV.t0 = ts;                                     % times in the CSV are relative to this
end
nm = cfg.agv.names;
pos = out.pos; spd = NaN;
if ~isempty(S.est) && ~out.stale, spd = S.est.speed; end

%% stale changes
if out.stale ~= EV.prevStale && ~EV.first
    if out.stale, e = 'STALE_START'; else, e = 'STALE_END'; end
    EV.ev(end+1, :) = row(ts, e, '', '', '', '', pos, spd, NaN, NaN, out.B);
end
EV.prevStale = out.stale;

%% fence changes (only judged with trustworthy data)
if ~out.stale && out.fenceOn ~= EV.prevFence
    if out.fenceOn
        EV.ev(end+1, :) = row(ts, 'FENCE_ENTER', '', '', '', '', pos, spd, NaN, NaN, out.B);
        EV.visit = struct('t0', ts, 'worst', 1, 'stops', 0, 'dMin', Inf, 'tWarn', NaN, 'tLane', NaN);
    else
        EV.ev(end+1, :) = row(ts, 'FENCE_EXIT', '', '', '', '', pos, spd, NaN, NaN, out.B);
        EV = closeVisit(EV, ts);
    end
    EV.prevFence = out.fenceOn;
end

%% level changes per AGV + reaction measurement
for k = 1:nA
    l = S.agv(k).lane; Ln = S.lane(l);
    d = out.desired(k);
    if d ~= EV.prevDesired(k) && ~EV.first
        if ~out.stale
            why = out.lane(l).reason; dl = out.lane(l).d; ttl = out.lane(l).ttl;
            if d < out.lane(l).level, why = 'has passed the person (E2)';
            elseif isempty(why), why = 'clear'; end
        else, why = 'fail-safe: data stale'; dl = NaN; ttl = NaN; end
        if ~isempty(EV.pending{k}), EV.pending{k} = []; end      % superseded: leave reaction blank
        EV.ev(end+1, :) = row(ts, 'LEVEL_CHANGE', S.agv(k).name, nm{EV.prevDesired(k)}, nm{d}, why, ...
                              pos, spd, dl, ttl, out.B);
        approaching = ~out.stale && (out.lane(l).sp - S.agv(k).s - cfg.agv.length / 2) > 0;
        EV.pending{k} = [size(EV.ev, 1), ts, d, NaN, approaching]; % row, command t, level, applied t, approaching
        if ~isempty(EV.visit)
            if d == 4, EV.visit.stops = EV.visit.stops + 1; end
            if d >= 2 && isnan(EV.visit.tWarn), EV.visit.tWarn = ts; end
        end
    end
    EV.prevDesired(k) = d;

    p = EV.pending{k};
    if ~isempty(p)
        if isnan(p(4)) && out.applied(k) == p(3)
            p(4) = ts; EV.ev{p(1), 12} = ts - p(2);                % latency
        end
        vT = cfg.agv.vMax * cfg.agv.levels(p(3));
        if ~isnan(p(4)) && abs(S.agv(k).v - vT) < 0.01
            EV.ev{p(1), 13} = ts - p(4);                           % settle time
            if p(3) == 4 && p(5) && ~out.stale                    % stop margin to crossing point
                EV.ev{p(1), 14} = out.lane(l).sp - S.agv(k).s - cfg.agv.length / 2;
            end
            p = [];
        end
        EV.pending{k} = p;
    end
end

%% visit statistics
if ~isempty(EV.visit) && ~out.stale
    EV.visit.worst = max(EV.visit.worst, out.overall);
    for k = 1:nA
        if S.agv(k).s >= 0 && S.agv(k).v > 0.05
            Ln = S.lane(S.agv(k).lane); c = Ln.a + S.agv(k).s * Ln.u;
            EV.visit.dMin = min(EV.visit.dMin, norm(c - pos));
        end
    end
    if isnan(EV.visit.tLane) && any([out.lane.d] <= 0), EV.visit.tLane = ts; end
end
EV.first = false;
EV.tLast = ts;
end

% -------------------------------------------------------------------------
function r = row(ts, ev, agv, from, to, why, pos, spd, dl, ttl, B)
% columns: 1 time 2 event 3 agv 4 from 5 to 6 reason 7 x 8 y 9 speed
%          10 lane_dist 11 ttl 12 latency 13 settle 14 margin 15 buffer
r = {ts, ev, agv, from, to, why, pos(1), pos(2), spd, dl, ttl, NaN, NaN, NaN, B};
end

function EV = closeVisit(EV, ts)
v = EV.visit;
if isempty(v), return; end
warn = v.tLane - v.tWarn;                           % NaN if never in a lane / never warned
EV.vis(end+1, :) = [v.t0, ts, ts - v.t0, v.worst, v.stops, v.dMin, warn];
EV.visit = [];
end
