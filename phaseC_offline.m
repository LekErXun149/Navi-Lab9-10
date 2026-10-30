%% phaseC_offline.m - run the full safety system on recordings (no hardware, no wall clock).
% For each recording: time each AGV spends at each level, number of STOP events,
% closest person-AGV distance (AGV on the lane, faster than CRAWL), a timeline figure, and the event
% log written to logs/<recording>_events.csv and _visits.csv.
if ~exist('doPlot', 'var'), doPlot = true; end
cfg = uwb_config();
ext = '.xlsx';
if ~(exist('readmatrix', 'file') || exist('readmatrix', 'builtin')), ext = '.csv'; end
recs = {'walk1_1704', 'walk2', 'static_1533'};
nm = cfg.agv.names;
col = [0.20 0.70 0.30; 1.00 0.75 0.00; 1.00 0.45 0.00; 0.85 0.10 0.10];

for r = 1:numel(recs)
    F  = nf2_load(fullfile('data', [recs{r} ext]), cfg.anchor.n);
    [Lg, EV] = safety_run_offline(F, cfg);
    S0 = safety_init(cfg); nA = numel(S0.agv); n = numel(Lg.t);
    fprintf('\n=== %s: %.0f s, fence INSIDE %.0f%% of the time, stale %.1f%%\n', ...
        recs{r}, Lg.t(end) - Lg.t(1), 100 * mean(Lg.fenceOn), 100 * mean(Lg.stale));
    fprintf('    AGV     NORMAL  SLOW  CRAWL  STOP | STOP events | closest to person (AGV faster than CRAWL)\n');
    for a = 1:nA
        Ln = S0.lane(S0.agv(a).lane);
        xy = Ln.a + Lg.s(:, a) * Ln.u;
        dd = hypot(xy(:, 1) - Lg.pos(:, 1), xy(:, 2) - Lg.pos(:, 2));
        mv = Lg.s(:, a) >= 0 & Lg.v(:, a) > cfg.agv.vMax * cfg.agv.levels(3);  % on the lane, faster than CRAWL
        dmin = min([dd(mv); Inf]);
        c = histc(Lg.applied(:, a), 1:4) / n * 100;
        ev = sum(diff([0; Lg.applied(:, a) == 4]) == 1);
        fprintf('    %-6s  %5.0f%% %4.0f%% %5.0f%% %4.0f%% | %6d      | %5.2f m\n', Lg.agvNames{a}, c, ev, dmin);
    end
    csv = safety_events_write(EV, fullfile('logs', recs{r}), cfg);
    stopRows = EV.ev(strcmp(EV.ev(:, 2), 'LEVEL_CHANGE') & strcmp(EV.ev(:, 5), 'STOP'), :);
    lat = [stopRows{:, 12}]; settle = [stopRows{:, 13}]; marg = [stopRows{:, 14}];
    fprintf('    events %d, fence visits %d | STOP: latency %.2f s, braking %.1f s (median), stop margin min %.1f m\n', ...
        size(EV.ev, 1), size(EV.vis, 1) + ~isempty(EV.visit), median(lat(isfinite(lat))), ...
        median(settle(isfinite(settle))), min([marg(isfinite(marg)) Inf]));
    fprintf('    -> %s, %s\n', csv{1}, csv{2});

    if doPlot
        figure('Name', ['Phase C - ' recs{r}], 'Color', 'w', 'Position', [80 80 1100 650]);
        try theme(gcf, 'light'); catch, end
        subplot(1, 2, 1); hold on; axis equal; grid on; box on;
        A = cfg.anchor.xy; Fz = cfg.site.fence;
        plot(A(:, 1), A(:, 2), 'k^', 'MarkerFaceColor', 'y');
        plot(Fz([1:end 1], 1), Fz([1:end 1], 2), 'r--');
        for l = 1:numel(S0.lane)
            plot([S0.lane(l).a(1) S0.lane(l).b(1)], [S0.lane(l).a(2) S0.lane(l).b(2)], '-', ...
                 'Color', [0.5 0.5 0.5], 'LineWidth', 6);
        end
        lv = max(Lg.laneLevel, [], 2);
        for v = 1:4
            m = lv == v & ~Lg.stale;
            plot(Lg.pos(m, 1), Lg.pos(m, 2), '.', 'Color', col(v, :), 'MarkerSize', 6);
        end
        xlabel('x (m)'); ylabel('y (m)'); title({strrep(recs{r}, '_', ' '), 'person coloured by worst lane level'});
        subplot(1, 2, 2); hold on; grid on; box on;
        tt = Lg.t - Lg.t(1);
        for a = 1:nA
            for v = 1:4
                m = Lg.applied(:, a) == v;
                plot(tt(m), a + 0 * tt(m), 's', 'Color', col(v, :), 'MarkerFaceColor', col(v, :), 'MarkerSize', 4);
            end
        end
        plot(tt(Lg.stale), 0.4 + 0 * tt(Lg.stale), 'k.', 'MarkerSize', 4);
        set(gca, 'YTick', [0.4 1:nA], 'YTickLabel', [{'stale'} Lg.agvNames], 'YLim', [0 nA + 0.6]);
        xlabel('time (s)'); title('Level each AGV acts on (green NORMAL, amber SLOW, orange CRAWL, red STOP)');
    end
end
