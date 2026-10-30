%% phaseC_run.m - AGV safety system: live tag (or replay) -> risk logic -> simulated AGVs.
% Before running live: close NAssistant, check cfg.serial.port, the anchors and
% cfg.site (fence + lanes) in uwb_config.m. Close the figure to stop.
% Saves logs/phaseC_<time>.mat, the event log logs/phaseC_<time>_events.csv and
% _visits.csv (open in Excel); in serial mode also the raw bytes logs/raw_<time>.bin.

MODE        = "serial";                 % "serial" (live tag) or "replay"
REPLAYFILE  = "data/walk2.xlsx";        % used when MODE = "replay"
REPLAYSPEED = 1;                        % 1 = real time, 2 = double speed

cfg = uwb_config();
if strcmp(MODE, "serial")
    src = UwbSource('serial', cfg); REPLAYSPEED = 1;
    fprintf('Live on %s @ %d baud. Raw log: %s\n', cfg.serial.port, cfg.serial.baud, src.rawFile);
else
    src = UwbSource('replay', cfg, REPLAYFILE, REPLAYSPEED);
    fprintf('Replaying %s at %gx\n', REPLAYFILE, REPLAYSPEED);
end
cleanup = onCleanup(@() src.close());           % always release the COM port

S = safety_init(cfg); st = []; EV = [];
H = safety_draw([], cfg);
nA = numel(S.agv); nMax = 200000; k = 0;        % log, preallocated
Lg.t = nan(nMax, 1); Lg.pos = nan(nMax, 2); Lg.stale = false(nMax, 1);
Lg.applied = ones(nMax, nA); Lg.v = nan(nMax, nA); Lg.overall = ones(nMax, 1);
trail = nan(750, 2);                            % last 30 s at 25 Hz
t0 = tic; tDraw = tic;

while ishandle(H.fig)
    if src.isDone(), break; end
    ts = toc(t0) * REPLAYSPEED;                 % simulation clock
    F = src.poll(); est = [];
    if ~isempty(F.t)
        [E, st] = uwb_process(F, st, cfg);
        est = struct('pos', E.pos(end, :), 'vel', E.vel(end, :), 'sigmaPos', E.sigmaPos(end), ...
                     'stale', E.stale(end), 'speed', E.speed(end));
        trail = [trail(numel(F.t) + 1:end, :); E.pos];
        trail = trail(max(1, end - 749):end, :);
    end
    [S, out] = safety_step(S, ts, est, cfg);
    EV = safety_events(EV, S, out, cfg);
    if k < nMax
        k = k + 1;
        Lg.t(k) = ts; Lg.stale(k) = out.stale; Lg.overall(k) = out.overall;
        Lg.applied(k, :) = out.applied; Lg.v(k, :) = out.v;
        if ~isempty(S.est), Lg.pos(k, :) = S.est.pos; end
    end
    if toc(tDraw) > 0.1, tDraw = tic; H = safety_draw(H, cfg, S, out, trail); end
    pause(0.02);
end

fn = fieldnames(Lg); for j = 1:numel(fn), Lg.(fn{j}) = Lg.(fn{j})(1:k, :); end
Lg.agvNames = {S.agv.name};
if ~exist('logs', 'dir'), mkdir('logs'); end
base = fullfile('logs', ['phaseC_' datestr(now, 'yyyymmdd_HHMMSS')]);
save([base '.mat'], 'Lg', 'EV', 'cfg');
csv = safety_events_write(EV, base, cfg);
fprintf('Stopped after %.0f s. Saved %s.mat, %s, %s\n', ts, base, csv{1}, csv{2});
for a = 1:nA
    c = histc(Lg.applied(:, a), 1:4) / max(k, 1) * 100;
    fprintf('  %-6s NORMAL %3.0f%%  SLOW %3.0f%%  CRAWL %3.0f%%  STOP %3.0f%%\n', Lg.agvNames{a}, c);
end
clear cleanup
