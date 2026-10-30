%% phaseD_run.m - Phase D: run every scenario, compare with the basic system, write results.
% Run from the Scenarios folder (or anywhere: it finds the project folder itself).
%
% Options - set before running, otherwise the defaults below are used:
%   OPT.quick     = true;                        % fast check: 2 runs, full + basic only
%   OPT.runs      = 10;                          % runs per scenario and noise profile
%   OPT.profiles  = {'typical', 'worst'};        % noise profiles (see phaseD_noise)
%   OPT.versions  = {'full','basic','noE1','noE2','noE3','noE5'};
%   OPT.scenarios = {'S5', 'S9'};                % subset; {} = all
%   OPT.clearance = 2.5;                         % fence-to-lane distance (m) for the test
%                                                % site; [] = use cfg.site.fence as it is
% Output (Scenarios/results/): DTEST_RESULTS.md, phaseD_metrics.csv, figures, phaseD_results.mat

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
root = fileparts(here);
addpath(root); addpath(here);
if ~exist('OPT', 'var'), OPT = struct(); end
def = struct('quick', false, 'runs', 10, 'profiles', {{'typical', 'worst'}}, ...
             'versions', {{'full', 'basic', 'noE1', 'noE2', 'noE3', 'noE5'}}, 'scenarios', {{}}, ...
             'clearance', 2.5);
fn = fieldnames(def);
for i = 1:numel(fn), if ~isfield(OPT, fn{i}), OPT.(fn{i}) = def.(fn{i}); end, end
if OPT.quick, OPT.runs = 2; OPT.versions = {'full', 'basic'}; end

cfg0 = phaseD_site(uwb_config(), OPT.clearance);
SC = phaseD_scenarios(cfg0);
if ~isempty(OPT.scenarios)
    SC = SC(cellfun(@(s) any(strcmp(s.id, OPT.scenarios)), SC));
end
nV = numel(OPT.versions);
cfgV = cell(1, nV);
for v = 1:nV, cfgV{v} = phaseD_policy(cfg0, OPT.versions{v}); end

% list of jobs: one per scenario x profile x run (replay: one per walk file)
jobs = {};
for s = 1:numel(SC)
    if strcmp(SC{s}.type, 'replay')
        for f = 1:numel(SC{s}.files), jobs(end+1, :) = {s, 'real', f}; end %#ok<SAGROW>
    else
        for p = 1:numel(OPT.profiles)
            for r = 1:OPT.runs, jobs(end+1, :) = {s, OPT.profiles{p}, r}; end %#ok<SAGROW>
        end
    end
end
if isempty(OPT.clearance), cl = 'fence from uwb_config.m'; else, cl = sprintf('fence %.1f m from the lanes', OPT.clearance); end
fprintf('Phase D: %d scenarios, %d jobs x %d versions, %s\n', numel(SC), size(jobs, 1), nV, cl);

rows = struct('sc', {}, 'id', {}, 'profile', {}, 'run', {}, 'version', {}, 'm', {});
t0 = tic;
for j = 1:size(jobs, 1)
    s = jobs{j, 1}; sc = SC{s}; prof = jobs{j, 2}; r = jobs{j, 3};
    seed = 1000 * s + 100 * find(strcmp(OPT.profiles, prof), 1, 'first') + r;
    if strcmp(sc.type, 'replay')
        [F, E, P] = phaseD_prepare(sc, '', 0, cfg0, root, r); sc.warm = 3;
    else
        [F, E, P] = phaseD_prepare(sc, prof, seed, cfg0, root);
    end
    for v = 1:nV
        m = phaseD_simulate(F, E, P, sc, cfgV{v});
        m.pass = sc.pass(m);
        rows(end+1) = struct('sc', s, 'id', sc.id, 'profile', prof, 'run', r, ...
                             'version', OPT.versions{v}, 'm', m); %#ok<SAGROW>
    end
    el = toc(t0);
    if mod(j, 5) == 0 || j == size(jobs, 1)
        fprintf('  %4d/%d jobs  %s  elapsed %.0f s, about %.0f s left\n', j, size(jobs, 1), sc.id, ...
                el, el / j * (size(jobs, 1) - j));
    end
end

outdir = fullfile(here, 'results');
if ~exist(outdir, 'dir'), mkdir(outdir); end
save(fullfile(outdir, 'phaseD_results.mat'), 'rows', 'OPT', 'cfg0');
phaseD_report(rows, SC, OPT, cfg0, outdir);
