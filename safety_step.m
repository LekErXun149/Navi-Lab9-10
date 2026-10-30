function [S, out] = safety_step(S, ts, est, cfg)
%SAFETY_STEP  One update of the AGV safety logic and AGV simulation.
%   [S, out] = safety_step(S, ts, est, cfg)
%     ts   simulation time (s), must increase
%     est  latest estimate since the last step, or [] if no new frame. Fields:
%          pos [x y], vel [vx vy], sigmaPos, stale (from uwb_process)
%
%   Levels: 1 NORMAL, 2 SLOW, 3 CRAWL, 4 STOP (higher = more cautious)
%   Per lane (person's distance d to the lane edge, buffer B):
%     B   = min(bufBase + bufK * sigmaPos, bufMax)                      (E3a; E3b via KF)
%     TTL = d / (speed towards the lane), only if that speed > vApproach  (E1)
%     STOP  if d < B+stopBand   or TTL < tStop
%     CRAWL if d < B+crawlBand
%     SLOW  if d < B+slowBand   or TTL < tSlow
%     (bands = standing-start zones from Phase B: inside, a person who starts
%      walking could reach the lane before a faster approaching AGV could stop)
%     Each condition switches ON at once and OFF only when it is
%     clear (distance: hystGap further out) for 'dwell' seconds          (E5)
%   Person outside the fence -> all lanes NORMAL.
%   Entry rule: for entryHold s after the person enters the fence, every lane
%   within entryReach is STOP (speed estimate not yet reliable after a turn).
%   Per AGV: only an AGV whose REAR has not yet passed the person's crossing
%   point by passMargin takes its lane's level; one that has passed stays NORMAL (E2)
%   No data for staleTime, filter stale, or sigmaPos > maxSigma -> every AGV
%   at least staleLevel, a STOP is kept                                  (E4)
%   cfg.policy (optional, Phase D comparisons): mode 'basic' = every AGV STOPs
%   whenever the person is inside the fence; E1/E2/E3/E5 = false switches that
%   enhancement off (E3 off -> fixed buffer cfg.policy.fixedBuf).
%   Commands reach the AGV after cfg.agv.latency; AGVs brake/accelerate at
%   cfg.agv.decel / accel, keep cfg.agv.minGap to the AGV in front (out.queued).
%   After the lane end an AGV returns out of sight and re-enters along a feeder
%   track of cfg.agv.feedLen before p0 (s < 0), where the rules already apply.

R = cfg.risk; nL = numel(S.lane); nA = numel(S.agv);
P = struct('mode', 'full', 'E1', true, 'E2', true, 'E3', true, 'E5', true, 'fixedBuf', 0.72);
if isfield(cfg, 'policy'), P = cfg.policy; end          % Phase D: basic system / switch off E1-E5
if ~P.E5, gapH = 0; dwellH = 0; else, gapH = R.hystGap; dwellH = R.dwell; end
if isnan(S.ts), dt = 0; else, dt = max(0, ts - S.ts); end
S.ts = ts;
if ~isempty(est), S.est = est; S.tsLastFrame = ts; end

stale = isempty(S.est) || S.est.stale || any(~isfinite(S.est.pos)) || ...
        (ts - S.tsLastFrame > cfg.health.staleTime) || ~(S.est.sigmaPos <= cfg.health.maxSigma);

out.ts = ts; out.stale = stale;
out.lane = struct('d', num2cell(nan(1, nL)), 'ttl', Inf, 'B', NaN, 'sp', NaN, ...
                  'level', 1, 'reason', '');
out.fenceOn = false; out.B = NaN; out.pos = [NaN NaN];

%% ---- warnings per lane (only with trustworthy data) ---------------------
if ~stale
    p = S.est.pos; v = S.est.vel;
    if P.E3, B = min(R.bufBase + R.bufK * S.est.sigmaPos, R.bufMax);
    else,    B = P.fixedBuf; end
    out.B = B; out.pos = p;
    [inside, dF] = geo_poly_dist(p, cfg.site.fence);
    [fenceOn, S.memFence] = risk_hyst(inside, ts, ~inside && dF > gapH, dwellH, S.memFence);
    out.fenceOn = fenceOn;
    if fenceOn && ~S.fenceWasOn, S.tFenceOn = ts; end           % entry time (entry rule)
    S.fenceWasOn = fenceOn;
    entry = fenceOn && (ts - S.tFenceOn) < R.entryHold && ~strcmp(P.mode, 'basic');
    for l = 1:nL
        [dc, sp, ~, n] = geo_seg(p, S.lane(l).a, S.lane(l).b);
        d = dc - cfg.site.laneWidth / 2;                   % to the lane edge (<0 = in the lane)
        vT = -dot(n, v);                                   % speed towards the lane
        if d <= 0, ttl = 0;
        elseif vT > R.vApproach && P.E1, ttl = d / vT;
        else, ttl = Inf; end
        % distance and time-to-lane warnings each have their own hysteresis (E5)
        cE   = entry && d < R.entryReach;                  % just entered, lane within walking reach
        zS = B + R.stopBand; zC = B + R.crawlBand; zW = B + R.slowBand;   % zone edges
        cond = [d < zS,        ttl < R.tStop,  d < zW,        ttl < R.tSlow,  cE,  d < zC];
        rel  = [d > zS + gapH, ttl >= R.tStop, d > zW + gapH, ttl >= R.tSlow, ~cE, d > zC + gapH];
        on = false(1, 6);
        for f = 1:6
            [on(f), S.lane(l).mem{f}] = risk_hyst(cond(f), ts, rel(f), dwellH, S.lane(l).mem{f});
        end
        lvlOf = [4 4 2 2 4 3];
        names = {'in lane / buffer', 'approaching fast', 'near lane', 'approaching', 'just entered', 'close to lane'};
        lvl = 1; why = '';
        if any(on)
            lvl = max(lvlOf(on));
            f = find(on & lvlOf == lvl);                    % flags giving this level
            g = f(cond(f));                                 % ...whose condition is true now
            if isempty(g), why = [names{f(1)} ' (holding)'];
            else, why = names{g(1)}; end
            if ~isempty(g) && any(g(1) == [2 4]), why = sprintf('%s, TTL %.1f s', why, ttl); end
        end
        if strcmp(P.mode, 'basic')                          % comparison: stop all inside the fence
            lvl = 1 + 3 * fenceOn; why = 'basic: person inside fence';
        end
        if ~fenceOn, lvl = 1; why = 'outside fence'; end
        out.lane(l).d = d; out.lane(l).ttl = ttl; out.lane(l).B = B; out.lane(l).sp = sp;
        out.lane(l).level = lvl; out.lane(l).reason = why;
    end
end

%% ---- level per AGV ------------------------------------------------------
for k = 1:nA
    l = S.agv(k).lane;
    if stale
        want = max(S.agv(k).desired, cfg.health.staleLevel);       % E4 (keeps a STOP)
    else
        ahead = out.lane(l).sp - (S.agv(k).s - cfg.agv.length / 2);   % person vs AGV REAR
        if ahead > -cfg.agv.passMargin || ~P.E2 || strcmp(P.mode, 'basic')
            want = out.lane(l).level;                              % E2: only if not yet passed
        else, want = 1; end
    end
    S.agv(k).desired = want;
    if want ~= S.agv(k).queue(end, 2), S.agv(k).queue(end+1, :) = [ts want]; end
    due = find(S.agv(k).queue(:, 1) <= ts - cfg.agv.latency, 1, 'last');
    S.agv(k).applied = S.agv(k).queue(due, 2);
    S.agv(k).queue = S.agv(k).queue(due:end, :);                   % drop old commands
end

%% ---- move AGVs (keep a safe gap to the AGV in front on the same lane) ---
sNow = [S.agv.s]; laneOf = [S.agv.lane];
out.queued = false(1, nA);
for k = 1:nA
    l = S.agv(k).lane; L = S.lane(l).L; Lloop = L + cfg.agv.feedLen;
    vT = cfg.agv.vMax * cfg.agv.levels(S.agv(k).applied);
    others = find(laneOf == l & (1:nA) ~= k);
    if ~isempty(others)
        gap = min(mod(sNow(others) - sNow(k), Lloop)) - cfg.agv.length;   % bumper to bumper
        vGap = sqrt(max(0, 2 * cfg.agv.decel * (gap - cfg.agv.minGap)));
        if vGap < vT, vT = vGap; out.queued(k) = true; end
    end
    if S.agv(k).v > vT, S.agv(k).v = max(vT, S.agv(k).v - cfg.agv.decel * dt);
    else,               S.agv(k).v = min(vT, S.agv(k).v + cfg.agv.accel * dt); end
    S.agv(k).s = S.agv(k).s + S.agv(k).v * dt;
    if S.agv(k).s > L, S.agv(k).s = S.agv(k).s - Lloop; end      % return via the feeder track
end

out.desired = [S.agv.desired];
out.applied = [S.agv.applied];
out.v       = [S.agv.v];
out.s       = [S.agv.s];
out.overall = max(out.applied);                                   % for the Leonardo (Phase E)
end
