%% phaseD_view.m - watch one Phase D scenario run with the live display.
% Shows the same display as phaseC_run, plus the TRUE person position (black x)
% and the true path (dashed). Useful to understand a result or to record a video.
% Change the settings below, then run. Close the figure to stop early.

SCENARIO  = 'S18';        % S1 ... S18 (S17 = real walk, see FILEIDX)
PROFILE   = 'typical';    % 'typical' or 'worst' noise
VERSION   = 'full';       % 'full', 'basic', 'noE1', 'noE2', 'noE3', 'noE5'
RUN       = 1;            % run number (same seed as phaseD_run uses)
SPEED     = 4;            % playback speed (1 = real time)
CLEARANCE = 2.5;          % fence-to-lane distance used by phaseD_run ([] = uwb_config fence)
FILEIDX   = 1;            % for S17: 1 = walk 1, 2 = walk 2

here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
root = fileparts(here); addpath(root); addpath(here);
cfg = phaseD_site(uwb_config(), CLEARANCE);
SC = phaseD_scenarios(cfg);
s = find(cellfun(@(x) strcmp(x.id, SCENARIO), SC)); sc = SC{s};
profiles = {'typical', 'worst'};
if strcmp(sc.type, 'replay')
    [F, E, P] = phaseD_prepare(sc, '', 0, cfg, root, FILEIDX); sc.warm = 3;
else
    seed = 1000 * s + 100 * find(strcmp(profiles, PROFILE)) + RUN;   % as in phaseD_run (default profiles)
    [F, E, P] = phaseD_prepare(sc, PROFILE, seed, cfg, root);
end
cfgV = phaseD_policy(cfg, VERSION);
[m, rec] = phaseD_simulate(F, E, P, sc, cfgV);        % result first (fast)...

% ...then replay it on screen, stepping the same logic again
dt = P.t(2) - P.t(1);
fIdx = zeros(numel(P.t), 1); fIdx(round(F.t / dt) + 1) = 1:numel(F.t);
S = safety_init(cfgV);
for k = 1:numel(S.agv), S.agv(k).v = cfgV.agv.vMax; S.agv(k).desired = 1; S.agv(k).applied = 1; S.agv(k).queue = [-Inf 1]; end
names = {S.agv.name}; tFence = phaseD_tfence(P, cfgV); kSet = find(P.t >= tFence - sc.agvLead, 1);
H = safety_draw([], cfgV);
set(H.fig, 'Name', sprintf('%s - %s (%s noise, %s)', sc.id, sc.name, PROFILE, VERSION));
plot(H.ax, P.xy(:, 1), P.xy(:, 2), 'k--', 'LineWidth', 0.8);
hTrue = plot(H.ax, NaN, NaN, 'kx', 'MarkerSize', 12, 'LineWidth', 2);
title(H.ax, sprintf('%s: %s  |  black x = TRUE position, blue = estimate', sc.id, sc.name), 'Interpreter', 'none');
trail = nan(750, 2); tWall = tic;
for k = 1:numel(P.t)
    if ~ishandle(H.fig), break; end
    if ~isempty(kSet) && k == kSet && isfield(sc, 'agv')
        for j = 1:size(sc.agv, 1)
            i = find(strcmp(names, sc.agv{j, 1}));
            S.agv(i).s = sc.agv{j, 2} - cfgV.agv.vMax * sc.agvLead; S.agv(i).v = cfgV.agv.vMax;
        end
    end
    est = [];
    if fIdx(k) > 0
        i = fIdx(k);
        est = struct('pos', E.pos(i, :), 'vel', E.vel(i, :), 'sigmaPos', E.sigmaPos(i), 'stale', E.stale(i), 'speed', E.speed(i));
        trail = [trail(2:end, :); E.pos(i, :)];
    end
    [S, out] = safety_step(S, P.t(k), est, cfgV);
    if mod(k, round(0.1 / dt)) == 0
        H = safety_draw(H, cfgV, S, out, trail);
        set(hTrue, 'XData', P.xy(k, 1), 'YData', P.xy(k, 2));
        while toc(tWall) < P.t(k) / SPEED, pause(0.005); end
    end
end
fprintf('%s %s %s run %d: near misses %d, contact %d, closest AGV faster than CRAWL %.2f m, productivity %.0f%%, pass %d\n', ...
    sc.id, PROFILE, VERSION, RUN, m.nearMiss, m.contact, m.minClear, m.prod, sc.pass(m));
