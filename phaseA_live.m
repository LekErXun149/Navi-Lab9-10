%% phaseA_live.m - live tag tracking (or replay of a recording) with display.
% Before running live: close NAssistant (it holds the COM port), check
% cfg.serial.port and the anchor coordinates in uwb_config.m.
% Close the figure window to stop. Everything is saved to logs/run_*.mat,
% and the raw serial bytes to logs/raw_*.bin (replayable with MODE = "replay").

MODE       = "serial";                     % "serial" (live tag) or "replay"
REPLAYFILE = "data/walk2.xlsx";            % used when MODE = "replay"
REPLAYSPEED = 1;                           % 1 = real time, 2 = double speed
if ~exist('doPlot', 'var'), doPlot = true; end

cfg = uwb_config();
if strcmp(MODE, "serial")
    src = UwbSource('serial', cfg);
    fprintf('Live on %s @ %d baud. Raw log: %s\n', cfg.serial.port, cfg.serial.baud, src.rawFile);
else
    src = UwbSource('replay', cfg, REPLAYFILE, REPLAYSPEED);
    fprintf('Replaying %s at %gx\n', REPLAYFILE, REPLAYSPEED);
end
cleanup = onCleanup(@() src.close());      % always release the COM port, even after Ctrl+C

state = []; Fall = nf2_empty(cfg.anchor.n); Eall = [];
wallLast = tic; tDraw = tic; nFrames = 0;

if doPlot
    fig = figure('Name', 'UWB live', 'Color', 'w', 'Position', [100 100 1000 560]);
    try theme(fig, 'light'); catch, end      % MATLAB R2025a+ dark theme hides the outline
    try, theme(fig, 'light'); catch, end      % MATLAB R2025a+ dark theme: keep this figure light
    axMap = axes('Position', [0.07 0.10 0.62 0.82], 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
    hold on; axis equal; grid on;
    A = cfg.anchor.xy;
    plot(A([1:end 1], 1), A([1:end 1], 2), 'k--');
    plot(A(:, 1), A(:, 2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
    for i = 1:cfg.anchor.n, text(A(i,1) + 0.2, A(i,2) + 0.3, sprintf('A%d', i-1)); end
    pad = 3; axis([min(A(:,1)) - pad, max(A(:,1)) + pad, min(A(:,2)) - pad, max(A(:,2)) + pad]);
    hTrail = plot(NaN, NaN, 'b-', 'LineWidth', 1);
    hFix   = plot(NaN, NaN, '.', 'Color', [1 .6 .3]);
    hCirc  = plot(NaN, NaN, 'r-');
    hPos   = plot(NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
    hVel   = plot(NaN, NaN, 'b-', 'LineWidth', 2);
    xlabel('x (m)'); ylabel('y (m)'); title('Tag: KF position, 2\sigma circle, 1 s velocity arrow');
    axTxt  = axes('Position', [0.72 0.10 0.26 0.82], 'Visible', 'off');
    hTxt   = text(0, 1, '', 'Parent', axTxt, 'FontName', 'Consolas', 'FontSize', 11, ...
                  'VerticalAlignment', 'top', 'Interpreter', 'none');
    th = linspace(0, 2*pi, 40);
end

while true
    if doPlot && ~ishandle(fig), break; end
    if src.isDone(), break; end
    F = src.poll();
    if isempty(F.t)
        if toc(wallLast) > cfg.health.staleTime && doPlot && ishandle(fig)
            set(hTxt, 'String', sprintf('NO DATA for %.1f s  ->  STALE (fail-safe)', toc(wallLast)), 'Color', 'r');
            drawnow;
        end
        pause(0.01); continue
    end
    wallLast = tic;
    [E, state] = uwb_process(F, state, cfg);
    Fall = nf2_concat(Fall, F);
    if isempty(Eall), Eall = E; else, Eall = nf2_concat(Eall, E); end
    nFrames = nFrames + numel(F.t);

    if doPlot && toc(tDraw) > 0.1                       % redraw at ~10 Hz
        tDraw = tic;
        k = numel(Eall.t); i0 = max(1, k - 750);        % last 30 s of trail
        p = Eall.pos(k, :); v = Eall.vel(k, :); s = Eall.sigmaPos(k);
        set(hTrail, 'XData', Eall.pos(i0:k, 1), 'YData', Eall.pos(i0:k, 2));
        set(hFix,   'XData', Eall.fix(max(1,k-50):k, 1), 'YData', Eall.fix(max(1,k-50):k, 2));
        set(hPos,   'XData', p(1), 'YData', p(2));
        set(hCirc,  'XData', p(1) + 2*s*cos(th), 'YData', p(2) + 2*s*sin(th));
        set(hVel,   'XData', p(1) + [0 v(1)], 'YData', p(2) + [0 v(2)]);
        if Eall.stale(k), col = 'r'; st = 'STALE'; else, col = 'k'; st = 'OK'; end
        set(hTxt, 'Color', col, 'String', sprintf([ ...
            'pos      (%6.2f, %6.2f) m\nspeed    %5.2f m/s\nheading  %6.0f deg\n' ...
            'sigma    %5.2f m\nresidual %5.2f m\nanchors  %d\ndata     %s\nframes   %d'], ...
            p(1), p(2), Eall.speed(k), Eall.heading(k), s, Eall.resRMS(k), ...
            Eall.nUsed(k), st, nFrames));
        drawnow;
    end
end

if ~exist('logs', 'dir'), mkdir('logs'); end
fn = fullfile('logs', ['run_' datestr(now, 'yyyymmdd_HHMMSS') '.mat']);
save(fn, 'Fall', 'Eall', 'cfg');
fprintf('Stopped. %d frames saved to %s\n', nFrames, fn);
if strcmp(MODE, "serial")
    fprintf('Decoder: %d good frames, %d bad checksums\n', src.stats.good, src.stats.badChecksum);
end
clear cleanup
