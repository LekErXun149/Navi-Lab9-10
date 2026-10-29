%% phaseA_offline.m - run the Phase A pipeline on the recorded data (no anchors needed).
% Produces the numbers Phase B uses to set thresholds:
%   - speed noise floor when the tag is still   (static files)
%   - position accuracy vs tape ground truth     (tag under A3)
%   - uncertainty multiplier k for the E3a buffer (95% coverage)
%   - smoothness / speed range while walking      (walk 1, walk 2)
% Put the four NAssistant exports in ./data (names below) and run.
if ~exist('doPlot', 'var'), doPlot = true; end
cfg = uwb_config();
ext = '.xlsx';
if ~(exist('readmatrix', 'file') || exist('readmatrix', 'builtin')), ext = '.csv'; end   % Octave

sets = { % file              type      truth [x y] (NaN = unknown)
    'static_1533'          'static'  [NaN NaN]
    'underA3_1620'         'static'  [12.093 0]
    'walk1_1704'           'walk'    [NaN NaN]
    'walk2'                'walk'    [NaN NaN] };

R = struct();
for s = 1:size(sets, 1)
    name = sets{s, 1}; kind = sets{s, 2}; truth = sets{s, 3};
    F = nf2_load(fullfile('data', [name ext]), cfg.anchor.n);
    [E, ~] = uwb_process(F, [], cfg);
    ok = isfinite(E.pos(:, 1));
    fprintf('\n=== %s (%s): %d frames, %.0f s, %d filter restarts, %.1f%% frames stale\n', ...
        name, kind, numel(F.t), sum(diff(F.t) .* (diff(F.t) > 0 & diff(F.t) < 1)), ...
        sum(E.reinit), 100 * mean(E.stale));

    spNA = hypot(F.velNA(:, 1), F.velNA(:, 2));
    fprintf('  speed  KF: mean %.2f  95%% %.2f m/s  | NAssistant vel: mean %.2f  95%% %.2f m/s\n', ...
        mean(E.speed(ok)), uwb_prctile(E.speed(ok), 95), mean(spNA), uwb_prctile(spNA, 95));
    fprintf('  sigmaPos (E3a): median %.2f m, 95%% %.2f m | residual RMS (E3b): median %.2f m\n', ...
        median(E.sigmaPos(ok)), uwb_prctile(E.sigmaPos(ok), 95), median(E.resRMS(isfinite(E.resRMS))));

    if strcmp(kind, 'static')
        fprintf('  position spread  raw fix: %.2f / %.2f m   KF: %.2f / %.2f m   (std x / y)\n', ...
            std(E.fix(isfinite(E.fix(:,1)), :)), std(E.pos(ok, :)));
    else
        L  = @(P) uwb_nansum(hypot(diff(P(:,1)), diff(P(:,2))));
        jt = @(P) 100 * sqrt(uwb_nanmean(sum(diff(P, 2).^2, 2)));
        fprintf('  path length  NAssistant %.0f m | raw fix %.0f m | KF %.0f m\n', L(F.posNA(:,1:2)), L(E.fix), L(E.pos));
        fprintf('  jitter       NAssistant %.1f cm | raw fix %.1f cm | KF %.1f cm\n', jt(F.posNA(:,1:2)), jt(E.fix), jt(E.pos));
    end

    if all(isfinite(truth))
        eNA  = hypot(F.posNA(:,1) - truth(1), F.posNA(:,2) - truth(2));
        eFix = hypot(E.fix(:,1) - truth(1), E.fix(:,2) - truth(2));
        eKF  = hypot(E.pos(:,1) - truth(1), E.pos(:,2) - truth(2));
        rm = @(e) sqrt(mean(e(isfinite(e)).^2));
        fprintf('  RMSE vs tape  NAssistant %.2f m | raw fix %.2f m | KF %.2f m\n', rm(eNA), rm(eFix), rm(eKF));
        kNeed = uwb_prctile(eKF(ok) ./ E.sigmaPos(ok), 95);
        fprintf('  E3a: buffer = k*sigmaPos covers true position 95%% of time when k = %.1f\n', kNeed);
        R.(name).kNeed = kNeed;
    end
    R.(name).E = E; R.(name).F = F;

    if doPlot
        fg = figure('Name', name, 'Color', 'w');
        try, theme(fg, 'light'); catch, end   % MATLAB R2025a+ dark theme: keep figures light
        subplot(2, 2, [1 3]); hold on; axis equal; grid on;
        plot(F.posNA(:,1), F.posNA(:,2), '.', 'Color', [.75 .75 .75], 'MarkerSize', 4);
        plot(E.fix(:,1), E.fix(:,2), '.', 'Color', [1 .6 .3], 'MarkerSize', 4);
        plot(E.pos(:,1), E.pos(:,2), 'b-', 'LineWidth', 1.2);
        A = cfg.anchor.xy; plot(A([1:end 1],1), A([1:end 1],2), 'k--^', 'MarkerFaceColor', 'y');
        if all(isfinite(truth)), plot(truth(1), truth(2), 'rp', 'MarkerSize', 16, 'MarkerFaceColor', 'r'); end
        legend('NAssistant', 'Own raw fix', 'Own KF', 'Anchors', 'Location', 'northwest');
        xlabel('x (m)'); ylabel('y (m)'); title(strrep(name, '_', ' '));
        tt = E.t - E.t(1);
        subplot(2, 2, 2); plot(tt, E.speed, 'b'); grid on; ylabel('KF speed (m/s)'); title('Speed');
        subplot(2, 2, 4); plot(tt, E.sigmaPos, 'r'); grid on; ylabel('\sigma_{pos} (m)'); xlabel('time (s)');
        title('Position uncertainty (E3a)');
    end
end
