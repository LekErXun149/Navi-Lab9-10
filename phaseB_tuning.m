%% phaseB_tuning.m - derive the risk thresholds from the recorded data.
% Every threshold in cfg.risk comes from a measurement here, so the report can
% justify each number. Prints a summary, draws the evidence figures, and writes
% TUNING_RESULTS.md. Run phaseA_offline first if you have not checked the data.
%
%   B1  speed noise when the tag is still        (evidence)
%   B2  walking speeds                            (evidence)
%   B5  error vs tape (tag under A3)              -> cfg.risk.bufBase, bufK, bufMax
%   B3  filter reaction time with REAL noise      \
%   B4  AGV stopping time + false triggers        -> cfg.risk.vApproach, tStop, tSlow
%   B6  flicker at a boundary when standing still -> cfg.risk.hystGap, dwell
%   B7  data gaps while walking                   -> cfg.health.staleTime
% To re-tune after a new anchor setup: record ~60 s standing still and one walk,
% then set before running, e.g.
%   files = {'data/new_static.xlsx', 'data/new_walk.xlsx'}; isStatic = [true false];
%   truthXY = [NaN NaN];   % or the tape position of the static recording
if ~exist('doPlot', 'var'), doPlot = true; end
cfg = uwb_config();
maxFalsePerMin = 1.0;   % accepted false SLOW triggers per minute (worst recording)
maxFlipsPerMin = 0.5;   % accepted flicker when standing on a boundary
ext = '.xlsx';
if ~(exist('readmatrix', 'file') || exist('readmatrix', 'builtin')), ext = '.csv'; end
ceilTo = @(x, s) ceil(x / s - 1e-9) * s;
if ~exist('files', 'var')       % default: the 26-Oct recordings
    files = strcat('data/', {'static_1533', 'underA3_1620', 'walk1_1704', 'walk2'}, ext);
    isStatic = [true true false false];
    truthXY = [NaN NaN; 12.093 0; NaN NaN; NaN NaN];   % tape position (static files)
end
for i = 1:numel(files)
    [~, D(i).name] = fileparts(files{i});
    D(i).F = nf2_load(files{i}, cfg.anchor.n);
    [D(i).E, ~] = uwb_process(D(i).F, [], cfg);
    % ignore the first 2 s after each filter (re)start: the velocity estimate is
    % still settling. Live, a restart only happens at start-up or after a data gap.
    E = D(i).E; settle = false(size(E.t)); r = find(E.reinit);
    for k = r', settle(k:end) = settle(k:end) | (E.t(k:end) - E.t(k) < 2 & E.t(k:end) >= E.t(k)); end
    D(i).ok = ~settle & isfinite(E.pos(:, 1));
end
fprintf('\n================ PHASE B TUNING ================\n');

%% B1 - speed noise when still -------------------------------------------
% A lane can lie in any direction, so the worst velocity COMPONENT matters.
fprintf('\nB1  Speed noise when the tag is still (KF)\n');
vc = [];
for i = find(isStatic)
    E = D(i).E; ok = D(i).ok;
    comp = abs([E.vel(ok, 1); E.vel(ok, 2)]);
    vc = [vc; comp]; %#ok<AGROW>
    fprintf('    %-13s speed p95 %.2f m/s | velocity component p99 %.2f, p99.5 %.2f m/s\n', ...
        D(i).name, uwb_prctile(E.speed(ok), 95), uwb_prctile(comp, 99), uwb_prctile(comp, 99.5));
end

%% B2 - walking speeds ----------------------------------------------------
fprintf('\nB2  Walking speeds (KF)\n');
for i = find(~isStatic)
    E = D(i).E; ok = isfinite(E.speed);
    fprintf('    %-13s median %.2f m/s, 95%% %.2f m/s, %.0f%% of frames above 0.3 m/s\n', ...
        D(i).name, median(E.speed(ok)), uwb_prctile(E.speed(ok), 95), 100 * mean(E.speed(ok) > 0.3));
end

%% B5 - safety buffer from ground truth -----------------------------------
fprintf('\nB5  Safety buffer vs tape ground truth\n');
gt = find(all(isfinite(truthXY), 2))';
if isempty(gt), error('B5 needs at least one static recording with a tape position (truthXY)'); end
e = []; s = [];
for i = gt
    E = D(i).E; ok = D(i).ok;
    e = [e; hypot(E.pos(ok, 1) - truthXY(i, 1), E.pos(ok, 2) - truthXY(i, 2))]; %#ok<AGROW>
    s = [s; E.sigmaPos(ok)]; %#ok<AGROW>
end
fprintf('    error: median %.2f m, 95%% %.2f m | sigmaPos median %.2f m\n', median(e), uwb_prctile(e, 95), median(s));
bases = 0:0.1:0.8; ks = [0 1 2 3]; cov = zeros(numel(bases), numel(ks));
for a = 1:numel(bases), for b = 1:numel(ks), cov(a, b) = mean(e <= bases(a) + ks(b) * s); end, end
fprintf('    coverage (true position inside buffer):\n      base\\k ');
fprintf('%6d', ks); fprintf('\n');
for a = 1:numel(bases), fprintf('      %4.1f m ', bases(a)); fprintf('%5.0f%%', 100 * cov(a, :)); fprintf('\n'); end
bufK = 2; aPick = find(cov(:, ks == bufK) >= 0.95, 1);
if isempty(aPick), aPick = numel(bases); end
bufBase = bases(aPick);
sc = arrayfun(@(i) D(i).E.sigmaPos(isfinite(D(i).E.sigmaPos)), 1:numel(D), 'UniformOutput', false); sAll = vertcat(sc{:});
bufMax = ceilTo(bufBase + bufK * uwb_prctile(sAll, 99), 0.25);
fprintf('    -> bufBase = %.1f m, bufK = %d (%.0f%% coverage), bufMax = %.2f m (base + %d x sigma p99)\n', ...
    bufBase, bufK, 100 * cov(aPick, ks == bufK), bufMax, bufK);
fprintf('    note: for the 26-Oct data this is the WORST case (corner, tag 0.74 m below anchors, NLOS)\n');

%% B3/B4 - approach threshold, reaction time and time-to-lane thresholds --
% Trade-off: a LOW vApproach detects approaching people earlier, but noise from a
% person standing still can then look like an approach and slow the AGVs.
% For each candidate vApproach:
%   B3  reaction time = when the filter first reports "approaching", using a
%       synthetic walk (0.8 m/s toward a lane from t = 2 s to 6 s, then stand)
%       with REAL position errors from both static recordings added (40 trials)
%   B4  tStop = detect + position lag + latency + brake from SLOW + margin
%       tSlow = tStop + latency + brake from NORMAL to SLOW
%   cost: false SLOW triggers per minute when standing still just outside the
%       distance zones (bufBase + slowBand from the lane edge), dwell applied
% Chosen: the lowest vApproach with <= maxFalsePerMin false triggers per minute.
% Missing a slow approach is safe: the distance zones still catch it.
fprintf('\nB3/B4  Approach threshold trade-off\n');
rc = arrayfun(@(i) D(i).E.resRMS(isfinite(D(i).E.resRMS)), find(isStatic), 'UniformOutput', false);
sig = hypot(cfg.kf.sigma0, cfg.kf.cRes * median(vertcat(rc{:})));
N = [];
for i = find(isStatic)
    fx = D(i).E.fix(all(isfinite(D(i).E.fix), 2), :);
    N = [N; fx - median(fx)]; %#ok<AGROW>
end
dt = 0.04; t = (0:dt:10)'; vWalk = 0.8; nT = 40;
truthY = vWalk * min(max(t - 2, 0), 4);
rand('seed', 3); %#ok<RAND>
VY = nan(numel(t), nT); PY = VY;                       % run the filter once per trial
for k = 1:nT
    o = randi(size(N, 1) - numel(t));
    z = [zeros(size(t)) truthY] + N(o + (1:numel(t)), :);
    kf = uwb_kf_init(cfg);
    for j = 1:numel(t)
        [kf, est] = uwb_kf_step(kf, t(j), z(j, :), sig);
        VY(j, k) = est.vel(2); PY(j, k) = est.pos(2);
    end
end
w = t > 4 & t < 6; posLag = median(mean(truthY(w) - PY(w, :)));
tLagPos = posLag / vWalk;
vN = cfg.agv.vMax * cfg.agv.levels(1); vS = cfg.agv.vMax * cfg.agv.levels(2);
vC = cfg.agv.vMax * cfg.agv.levels(3);
margin = 0.5; dwell = 1.0;
sTyp = median(D(1).E.sigmaPos(D(1).ok));                % typical position uncertainty
cands = 0.15:0.05:0.60; nc = numel(cands);
T = struct('vA', num2cell(cands)); 
for c = 1:nc
    vA = cands(c); lag = nan(nT, 1); lagS = nan(nT, 1);
    for k = 1:nT
        a1 = find(t > 2 & VY(:, k) > vA, 1); if ~isempty(a1), lag(k) = t(a1) - 2; end
        b1 = find(t > 6 & VY(:, k) < vA, 1); if ~isempty(b1), lagS(k) = t(b1) - 6; end
    end
    T(c).tFilter  = uwb_prctile(lag, 95);
    T(c).tStopDet = uwb_prctile(lagS, 95);
    T(c).tStopRaw = T(c).tFilter + tLagPos + cfg.agv.latency + vS / cfg.agv.decel + margin;
    T(c).tSlowRaw = T(c).tStopRaw + cfg.agv.latency + (vN - vS) / cfg.agv.decel;
    T(c).tStop = ceilTo(T(c).tStopRaw, 0.5); T(c).tSlow = ceilTo(T(c).tSlowRaw, 0.5);
    % Standing-start zones (Phase D, S8/S18): a person standing at distance d can
    % reach the lane in d / vWalk. An AGV at level speed v needs
    %   tReact + v / decel,  tReact = detect + position lag + latency + margin.
    % So within reach(v) = vWalk x (tReact + v / decel) that speed is unsafe and the
    % approaching AGV must already be one level slower. Zone edges are measured from
    % the lane edge; the bands are beyond the typical buffer (B = bufBase + bufK sigma).
    tReact = T(c).tFilter + tLagPos + cfg.agv.latency + margin;
    reachN = vWalk * (tReact + vN / cfg.agv.decel);         % NORMAL unsafe inside -> SLOW
    reachS = vWalk * (tReact + vS / cfg.agv.decel);         % SLOW unsafe inside   -> CRAWL
    reachC = vWalk * (tReact + vC / cfg.agv.decel);         % CRAWL unsafe inside  -> STOP
    Btyp = bufBase + bufK * sTyp;
    T(c).slowBand  = ceilTo(reachN - Btyp, 0.1);
    T(c).crawlBand = ceilTo(reachS - Btyp, 0.1);
    T(c).stopBand  = max(0, ceilTo(reachC - Btyp, 0.1));
    T(c).reach = [reachN reachS reachC];
    % each candidate is tested standing just outside its own SLOW zone
    D0 = bufBase + T(c).slowBand; T(c).D0 = D0;
    worst = 0;                                          % false SLOW triggers / min
    for i = find(isStatic)
        E = D(i).E; ok = D(i).ok; tt = E.t(ok);
        mins = sum(diff(tt) .* (diff(tt) > 0 & diff(tt) < 1)) / 60;
        for ax = 1:2
            for sg = [-1 1]
                va = sg * E.vel(ok, ax);
                trig = va > vA & D0 ./ max(va, eps) < T(c).tSlow;
                mem = []; prev = false; n = 0;
                for j = 1:numel(tt)
                    [on, mem] = risk_hyst(trig(j), tt(j), ~trig(j), dwell, mem);
                    n = n + (on && ~prev); prev = on;
                end
                worst = max(worst, n / mins);
            end
        end
    end
    T(c).falsePerMin = worst;
    wc = arrayfun(@(i) D(i).E.speed(isfinite(D(i).E.speed)), find(~isStatic), 'UniformOutput', false); wv = vertcat(wc{:});
    T(c).walkDetected = mean(wv > vA);
end
fprintf('    standing still just outside the SLOW zone; walk detection = %% of walking frames faster than vApproach\n');
fprintf('    vApproach | detect 95%% | tStop | tSlow | slow/crawl/stop band | stand at | false SLOW/min | walking detected\n');
for c = 1:nc
    fprintf('      %.2f    |   %.2f s   | %.1f s | %.1f s |  %.1f / %.1f / %.1f m  |  %.1f m   |     %5.2f      |     %3.0f%%\n', ...
        cands(c), T(c).tFilter, T(c).tStop, T(c).tSlow, T(c).slowBand, T(c).crawlBand, T(c).stopBand, ...
        T(c).D0, T(c).falsePerMin, 100 * T(c).walkDetected);
end
cPick = find([T.falsePerMin] <= maxFalsePerMin, 1); if isempty(cPick), cPick = nc; end
vApproach = cands(cPick); tFilter = T(cPick).tFilter;
tStop = T(cPick).tStop; tSlow = T(cPick).tSlow; tStopRaw = T(cPick).tStopRaw; tSlowRaw = T(cPick).tSlowRaw;
fprintf('    -> vApproach = %.2f m/s (lowest with <= %.1f false SLOW per minute)\n', vApproach, maxFalsePerMin);
fprintf('    tStop = detect %.2f + position lag %.2f + latency %.2f + brake from SLOW %.2f + margin %.2f = %.2f -> %.1f s\n', ...
    tFilter, tLagPos, cfg.agv.latency, vS / cfg.agv.decel, margin, tStopRaw, tStop);
fprintf('    tSlow = tStop %.2f + latency %.2f + brake NORMAL->SLOW %.2f = %.2f -> %.1f s\n', ...
    tStopRaw, cfg.agv.latency, (vN - vS) / cfg.agv.decel, tSlowRaw, tSlow);
fprintf('    at a walk of 0.8 m/s: STOP from %.1f m, SLOW from %.1f m before the lane edge\n', 0.8 * tStop, 0.8 * tSlow);
reach = vWalk * tStop; entryReach = reach;
slowBand = T(cPick).slowBand; crawlBand = T(cPick).crawlBand; stopBand = T(cPick).stopBand;
rr = T(cPick).reach; Btyp = bufBase + bufK * sTyp;
fprintf('    standing-start zones (0.8 m/s x reaction %.2f s + braking), typical buffer %.2f m:\n', ...
    T(cPick).tFilter + tLagPos + cfg.agv.latency + margin, Btyp);
fprintf('      NORMAL unsafe within %.2f m -> slowBand  = %.1f m\n', rr(1), slowBand);
fprintf('      SLOW   unsafe within %.2f m -> crawlBand = %.1f m\n', rr(2), crawlBand);
fprintf('      CRAWL  unsafe within %.2f m -> stopBand  = %.1f m\n', rr(3), stopBand);
fprintf('    entry rule reach = walking reach in tStop = %.1f m\n', entryReach);

%% B6 - hysteresis: flicker when standing still on a boundary ------------
fprintf('\nB6  Flicker when standing on a boundary (warning releases per minute, worst case)\n');
gaps = 0:0.05:0.6; dwells = [0 0.5 1.0];
flipsKF = zeros(numel(gaps), numel(dwells)); flipsNA = zeros(numel(gaps), numel(dwells));
for i = find(isStatic)
    E = D(i).E; F = D(i).F; ok = D(i).ok & isfinite(F.posNA(:, 1));
    tt = E.t(ok); mins = sum(diff(tt) .* (diff(tt) > 0 & diff(tt) < 1)) / 60;
    for ax = 1:2
        for src = 1:2
            if src == 1, p = E.pos(ok, ax); else, p = F.posNA(ok, ax); end
            for off = [-0.1 0 0.1]
                bnd = median(p) + off;
                seg = cumsum([1; diff(tt) <= 0 | diff(tt) > 1]);   % split at time jumps
                for g = 1:numel(gaps)
                    for w = 1:numel(dwells)
                        n = 0;
                        for sgm = 1:max(seg)
                            jj = find(seg == sgm); mem = []; prev = NaN;
                            for j = jj'
                                [on, mem] = risk_hyst(p(j) < bnd, tt(j), p(j) > bnd + gaps(g), dwells(w), mem);
                                if ~isnan(prev), n = n + (prev && ~on); end   % count releases (ON -> OFF)
                                prev = on;
                            end
                        end
                        if src == 1, flipsKF(g, w) = max(flipsKF(g, w), n / mins);
                        else,        flipsNA(g, w) = max(flipsNA(g, w), n / mins); end
                    end
                end
            end
        end
    end
end
fprintf('      gap   | NAssistant dwell 0 / 0.5 / 1 s | own KF dwell 0 / 0.5 / 1 s\n');
for g = 1:2:numel(gaps)
    fprintf('      %.2f m | %7.1f %7.1f %7.1f        | %6.1f %6.1f %6.1f\n', gaps(g), flipsNA(g, :), flipsKF(g, :));
end
gPick = find(flipsKF(:, dwells == dwell) <= maxFlipsPerMin, 1);
if isempty(gPick), gPick = numel(gaps); end
hystGap = gaps(gPick);
fprintf('    -> hystGap = %.2f m, dwell = %.1f s (own KF: %.1f flips/min; NAssistant, no hysteresis: %.0f flips/min)\n', ...
    hystGap, dwell, flipsKF(gaps == hystGap, dwells == dwell), flipsNA(1, 1));

%% B7 - data gaps while walking ------------------------------------------
fprintf('\nB7  Data gaps while walking\n');
for i = find(~isStatic)
    E = D(i).E; a = E.age(isfinite(E.age));
    fprintf('    %-13s max time without a good update %.2f s, stale %.1f%% of frames, <3 anchors in %d frames\n', ...
        D(i).name, max(a), 100 * mean(E.stale), sum(E.nUsed < 3));
end
fprintf('    -> staleTime %.1f s is kept (no false fail-safe while walking)\n', cfg.health.staleTime);

%% Summary + results file -------------------------------------------------
rec = struct('vApproach', vApproach, 'tStop', tStop, 'tSlow', tSlow, 'bufBase', bufBase, ...
             'bufK', bufK, 'bufMax', bufMax, 'slowBand', slowBand, 'crawlBand', crawlBand, ...
             'stopBand', stopBand, 'entryReach', entryReach, ...
             'hystGap', hystGap, 'dwell', dwell);
fn = fieldnames(rec);
fprintf('\nRecommended vs current uwb_config.m:\n');
same = true;
for j = 1:numel(fn)
    cur = cfg.risk.(fn{j}); same = same && abs(cur - rec.(fn{j})) < 1e-9;
    fprintf('    cfg.risk.%-10s recommended %5.2f   current %5.2f %s\n', fn{j}, rec.(fn{j}), cur, ...
        ternary_str(abs(cur - rec.(fn{j})) < 1e-9, '', '  <-- update'));
end
if same, fprintf('    uwb_config.m is up to date.\n'); end

fid = fopen('TUNING_RESULTS.md', 'w');
fprintf(fid, '# Phase B - tuning results\n\nGenerated by `phaseB_tuning.m` on %s from: %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM'), strjoin({D.name}, ', '));
fprintf(fid, '| Setting | Value | Evidence |\n|---|---|---|\n');
fprintf(fid, '| `vApproach` | %.2f m/s | lowest value with <= %.1f false SLOW per minute standing still %.1f m from a lane, just outside the SLOW zone (%.2f/min measured); %.0f%% of walking frames are faster (B3/B4) |\n', ...
    vApproach, maxFalsePerMin, T(cPick).D0, T(cPick).falsePerMin, 100 * T(cPick).walkDetected);
fprintf(fid, '| `tStop` | %.1f s | detect %.2f + position lag %.2f + latency %.2f + brake from SLOW %.2f + margin %.2f s (B3, B4) |\n', tStop, tFilter, tLagPos, cfg.agv.latency, vS / cfg.agv.decel, margin);
fprintf(fid, '| `tSlow` | %.1f s | tStop + latency + brake NORMAL to SLOW %.2f s (B4) |\n', tSlow, (vN - vS) / cfg.agv.decel);
fprintf(fid, '| `bufBase`, `bufK` | %.1f m + %d x sigma | covers the true position %.0f%% of the time at the worst-case point (B5) |\n', bufBase, bufK, 100 * cov(aPick, ks == bufK));
fprintf(fid, '| `bufMax` | %.2f m | bufBase + %d x 99th percentile of sigma (B5) |\n', bufMax, bufK);
fprintf(fid, '| `slowBand`, `crawlBand`, `stopBand` | %.1f, %.1f, %.1f m | standing-start zones: a person standing within %.2f / %.2f / %.2f m of the lane can reach it before an approaching AGV at NORMAL / SLOW / CRAWL can stop (0.8 m/s x (detect + latency + margin + braking)), so the AGV must already be one level slower; bands are beyond the typical buffer %.2f m (B4, Phase D S8, S18) |\n', ...
    slowBand, crawlBand, stopBand, rr(1), rr(2), rr(3), Btyp);
fprintf(fid, '| `entryReach` | %.1f m | same walking reach: lanes this close STOP for `entryHold` s after the person enters the fence (Phase D S18) |\n', entryReach);
fprintf(fid, '| `hystGap`, `dwell` | %.2f m, %.1f s | %.1f flips/min standing on a boundary vs %.0f flips/min for NAssistant without hysteresis (B6) |\n', ...
    hystGap, dwell, flipsKF(gaps == hystGap, dwells == dwell), flipsNA(1, 1));
fprintf(fid, '| `staleTime` | %.1f s | no false fail-safe in either walk (B7) |\n', cfg.health.staleTime);
fprintf(fid, '\nAt a walk of 0.8 m/s: STOP starts %.1f m and SLOW %.1f m before the lane edge.\n', 0.8 * tStop, 0.8 * tSlow);
fprintf(fid, '\nLimitations: ground truth from %d static point(s); AGV braking and latency are assumed values in `cfg.agv`. Re-run after a new anchor setup.\n', numel(gt));
fclose(fid);
fprintf('\nWrote TUNING_RESULTS.md\n');

%% Figures ---------------------------------------------------------------
if doPlot
    figure('Name', 'Phase B tuning', 'Color', 'w', 'Position', [80 80 1100 700]);
    try theme(gcf, 'light'); catch, end
    subplot(2, 2, 1);
    edges = 0:0.02:1.4; hold on;
    histc_n = @(x) histc(x, edges) / numel(x);
    stairs(edges, histc_n(vc), 'b', 'LineWidth', 1.2);
    wc = arrayfun(@(i) D(i).E.speed(isfinite(D(i).E.speed)), find(~isStatic), 'UniformOutput', false); wv = vertcat(wc{:});
    stairs(edges, histc_n(wv), 'Color', [0.9 0.45 0.1], 'LineWidth', 1.2);
    plot([vApproach vApproach], ylim, 'k--', 'LineWidth', 1.2);
    legend('still: velocity component', 'walking: speed', 'vApproach', 'Location', 'northeast');
    xlabel('m/s'); ylabel('fraction of frames'); grid on; title('B1/B2  Still vs walking');

    subplot(2, 2, 2); hold on; grid on;
    plot(t, truthY, 'k-', 'LineWidth', 1.5);
    plot(t, PY(:, end), 'b-'); plot(t, VY(:, end), 'r-');
    plot(xlim, [vApproach vApproach], 'r--');
    legend('true y (m)', 'KF y (m)', 'KF v_y (m/s)', 'vApproach', 'Location', 'northwest');
    xlabel('time (s)'); title('B3  Reaction to start / stop (one trial)');

    subplot(2, 2, 3); hold on; grid on;
    cols = lines(numel(ks));
    for b = 1:numel(ks), plot(bases, 100 * cov(:, b), '-o', 'Color', cols(b, :)); end
    plot(xlim, [95 95], 'k--'); plot(bufBase, 100 * cov(aPick, ks == bufK), 'kp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');
    legend(arrayfun(@(k) sprintf('k = %d', k), ks, 'UniformOutput', false), 'Location', 'southeast');
    xlabel('bufBase (m)'); ylabel('coverage (%)'); title('B5  Buffer = base + k\sigma vs tape');

    subplot(2, 2, 4); hold on; grid on;
    plot(gaps, flipsNA(:, 1), 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
    for w = 1:numel(dwells), plot(gaps, flipsKF(:, w), '-o'); end
    plot([hystGap hystGap], ylim, 'k--');
    legend('NAssistant, no dwell', 'own KF, dwell 0 s', 'own KF, dwell 0.5 s', 'own KF, dwell 1 s', 'chosen gap');
    xlabel('hysteresis gap (m)'); ylabel('releases per minute'); title('B6  Flicker standing on a boundary');
end
