%% phaseD_clearance.m - how far must the fence be from the lanes? (design study)
% Runs the crossing scenarios with the fence at several distances from the lanes,
% for this system and the basic one, and reports near misses and warning time.
% Output: Scenarios/results/CLEARANCE_RESULTS.md and phaseD_clearance.png
if ~exist('OPTC', 'var'), OPTC = struct(); end
if ~isfield(OPTC, 'clear'),    OPTC.clear = [1.0 1.5 2.0 2.5 3.0]; end
if ~isfield(OPTC, 'runs'),     OPTC.runs = 5; end
if ~isfield(OPTC, 'ids'),      OPTC.ids = {'S5', 'S6', 'S7'}; end
if ~isfield(OPTC, 'versions'), OPTC.versions = {'full', 'basic'}; end
here = fileparts(mfilename('fullpath')); if isempty(here), here = pwd; end
root = fileparts(here); addpath(root); addpath(here);
profiles = {'typical', 'worst'};
R = struct('c', {}, 'id', {}, 'v', {}, 'm', {});
t0 = tic;
for c = OPTC.clear
    cfgC = phaseD_site(uwb_config(), c);
    SC = phaseD_scenarios(cfgC);
    for s = 1:numel(SC)
        sc = SC{s}; if ~any(strcmp(sc.id, OPTC.ids)), continue; end
        for p = 1:2
            for r = 1:OPTC.runs
                [F, E, P] = phaseD_prepare(sc, profiles{p}, 5000 + 100 * s + 10 * p + r, cfgC, root);
                for v = 1:numel(OPTC.versions)
                    m = phaseD_simulate(F, E, P, sc, phaseD_policy(cfgC, OPTC.versions{v}));
                    R(end+1) = struct('c', c, 'id', sc.id, 'v', OPTC.versions{v}, 'm', m); %#ok<SAGROW>
                end
            end
        end
    end
    fprintf('  clearance %.1f m done (%.0f s)\n', c, toc(t0));
end
outdir = fullfile(here, 'results'); if ~exist(outdir, 'dir'), mkdir(outdir); end
M = [R.m];
fid = fopen(fullfile(outdir, 'CLEARANCE_RESULTS.md'), 'w');
fprintf(fid, '# Fence clearance study\n\nScenarios %s, %d runs x 2 noise profiles per clearance.\n', strjoin(OPTC.ids, ', '), OPTC.runs);
fprintf(fid, 'Near miss = AGV faster than CRAWL within 0.5 m; contact = moving AGV touches the person.\n\n');
fprintf(fid, '| Clearance (m) | Scenario | Near misses this / basic | Contacts this / basic | Warning before lane this / basic (s) | Productivity this / basic (%%) |\n|---|---|---|---|---|---|\n');
for c = OPTC.clear
    for i = 1:numel(OPTC.ids)
        a = [R.c] == c & strcmp({R.id}, OPTC.ids{i}) & strcmp({R.v}, 'full');
        b = [R.c] == c & strcmp({R.id}, OPTC.ids{i}) & strcmp({R.v}, 'basic');
        wa = [M(a).warn]; wb = [M(b).warn];
        fprintf(fid, '| %.1f | %s | %d / %d | %d / %d | %.2f / %.2f | %.1f / %.1f |\n', c, OPTC.ids{i}, ...
            sum([M(a).nearMiss]), sum([M(b).nearMiss]), sum([M(a).contact]), sum([M(b).contact]), ...
            mean(wa(isfinite(wa))), mean(wb(isfinite(wb))), mean([M(a).prod]), mean([M(b).prod]));
    end
end
nm = zeros(numel(OPTC.clear), 2);
for k = 1:numel(OPTC.clear)
    nm(k, 1) = sum([M([R.c] == OPTC.clear(k) & strcmp({R.v}, 'full')).nearMiss]);
    nm(k, 2) = sum([M([R.c] == OPTC.clear(k) & strcmp({R.v}, 'basic')).nearMiss]);
end
ok = find(nm(:, 1) == 0, 1);
if ~isempty(ok)
    fprintf(fid, '\n**Smallest clearance with no near miss for this system: %.1f m.**\n', OPTC.clear(ok));
end
fclose(fid);
fig = figure('Color', 'w', 'Visible', 'off'); try theme(fig, 'light'); catch, end
bar(OPTC.clear, nm); grid on; legend('this system', 'basic', 'Location', 'northeast');
xlabel('fence to lane edge (m)'); ylabel(sprintf('near misses (%s)', strjoin(OPTC.ids, ', ')));
title('How much room the fence needs');
print(fig, fullfile(outdir, 'phaseD_clearance.png'), '-dpng', '-r110'); close(fig);
type(fullfile(outdir, 'CLEARANCE_RESULTS.md'));
