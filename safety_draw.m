function H = safety_draw(H, cfg, S, out, trail)
%SAFETY_DRAW  Create (H = []) or update the Phase C display.
%   H = safety_draw([], cfg)                    create the figure, returns handles
%   H = safety_draw(H, cfg, S, out, trail)      update; trail = Nx2 recent positions
%   Shows anchors, fence, lanes coloured by lane level, AGVs coloured by the
%   level they are acting on, the person with buffer circle and velocity arrow,
%   and a status panel.

col = [0.20 0.70 0.30; 1.00 0.75 0.00; 1.00 0.45 0.00; 0.85 0.10 0.10];   % NORMAL SLOW CRAWL STOP
if isempty(H)
    H.fig = figure('Name', 'UWB AGV safety', 'Color', 'w', 'Position', [60 60 1200 640]);
    try theme(H.fig, 'light'); catch, end
    H.ax = axes('Position', [0.05 0.08 0.62 0.86]); hold on; axis equal; grid on; box on;
    A = cfg.anchor.xy; Fz = cfg.site.fence;
    plot(A(:, 1), A(:, 2), 'k^', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
    for i = 1:size(A, 1), text(A(i, 1) + 0.2, A(i, 2) + 0.35, sprintf('A%d', i - 1)); end
    H.fence = plot(Fz([1:end 1], 1), Fz([1:end 1], 2), 'r--', 'LineWidth', 1.5);
    S0 = safety_init(cfg); w = cfg.site.laneWidth / 2;
    for l = 1:numel(S0.lane)
        a = S0.lane(l).a; b = S0.lane(l).b; m = S0.lane(l).nrm;
        P = [a + w*m; b + w*m; b - w*m; a - w*m];
        H.lane(l) = patch(P(:, 1), P(:, 2), col(1, :), 'FaceAlpha', 0.25, 'EdgeColor', 'none');
        q = a - 0.15 * S0.lane(l).u;                         % label just before the lane start
        if S0.lane(l).u(1) >= 0, arr = '\rightarrow'; al = 'right'; else, arr = '\leftarrow'; al = 'left'; end
        text(q(1), q(2), [cfg.site.lanes(l).name ' ' arr], 'FontSize', 8, ...
             'HorizontalAlignment', al, 'Color', [0.3 0.3 0.3]);
    end
    for k = 1:numel(S0.agv)
        H.agv(k) = patch(NaN(4, 1), NaN(4, 1), col(1, :), 'EdgeColor', 'k');
        H.agvTxt(k) = text(NaN, NaN, S0.agv(k).name, 'FontSize', 8, 'HorizontalAlignment', 'center');
    end
    H.trail = plot(NaN, NaN, 'b-');
    H.buf   = plot(NaN, NaN, 'b:', 'LineWidth', 1.2);
    H.vel   = plot(NaN, NaN, 'b-', 'LineWidth', 2);
    H.pos   = plot(NaN, NaN, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 9);
    pad = 2; ax = [min(A(:, 1)) - pad, max(A(:, 1)) + pad, min(A(:, 2)) - pad, max(A(:, 2)) + pad];
    axis(ax); xlabel('x (m)'); ylabel('y (m)');
    title('Person (blue, dotted = safety buffer), lanes and AGVs coloured by level');
    H.axTxt = axes('Position', [0.70 0.08 0.29 0.86], 'Visible', 'off');
    H.txt = text(0, 1, '', 'Parent', H.axTxt, 'FontName', 'Consolas', 'FontSize', 10, ...
                 'VerticalAlignment', 'top', 'Interpreter', 'none');
    H.th = linspace(0, 2*pi, 48);
    return
end
if ~ishandle(H.fig), return; end

% lanes and AGVs
for l = 1:numel(S.lane), set(H.lane(l), 'FaceColor', col(out.lane(l).level, :)); end
hl = cfg.agv.length / 2; hw = cfg.agv.width / 2;
for k = 1:numel(S.agv)
    Ln = S.lane(S.agv(k).lane); c = Ln.a + S.agv(k).s * Ln.u;
    P = [c + hl*Ln.u + hw*Ln.nrm; c + hl*Ln.u - hw*Ln.nrm; c - hl*Ln.u - hw*Ln.nrm; c - hl*Ln.u + hw*Ln.nrm];
    set(H.agv(k), 'XData', P(:, 1), 'YData', P(:, 2), 'FaceColor', col(S.agv(k).applied, :));
    set(H.agvTxt(k), 'Position', [c + (hw + 0.25) * Ln.nrm 0]);
end

% person
if ~isempty(trail), set(H.trail, 'XData', trail(:, 1), 'YData', trail(:, 2)); end
if ~out.stale
    p = out.pos; v = S.est.vel;
    set(H.pos, 'XData', p(1), 'YData', p(2), 'MarkerFaceColor', 'b');
    set(H.buf, 'XData', p(1) + out.B * cos(H.th), 'YData', p(2) + out.B * sin(H.th));
    set(H.vel, 'XData', p(1) + [0 v(1)], 'YData', p(2) + [0 v(2)]);
else
    set(H.pos, 'MarkerFaceColor', 'w'); set(H.buf, 'XData', NaN, 'YData', NaN);
end

% status panel
nm = cfg.agv.names;
s = sprintf('time      %7.1f s\n', out.ts);
if out.stale, s = [s sprintf('DATA      STALE -> fail-safe (>= %s)\n', nm{cfg.health.staleLevel})];
else
    s = [s sprintf('person    (%5.2f, %5.2f) m\n', out.pos)];
    s = [s sprintf('speed     %5.2f m/s\n', S.est.speed)];
    s = [s sprintf('buffer    %5.2f m\n', out.B)];
    if out.fenceOn, f = 'INSIDE'; else, f = 'outside'; end
    s = [s sprintf('fence     %s\n', f)];
end
s = [s sprintf('\nLANE    dist   TTL   level\n')];
for l = 1:numel(S.lane)
    tt = out.lane(l).ttl; if isinf(tt), tts = '  -  '; else, tts = sprintf('%4.1fs', tt); end
    s = [s sprintf('%-7s %5.2f %s  %s\n  %s\n', cfg.site.lanes(l).name, out.lane(l).d, tts, ...
        nm{out.lane(l).level}, out.lane(l).reason)];
end
s = [s sprintf('\nAGV     level    speed\n')];
for k = 1:numel(S.agv)
    q = ''; if out.queued(k), q = ' queued'; end
    s = [s sprintf('%-6s  %-7s %4.2f m/s%s\n', S.agv(k).name, nm{S.agv(k).applied}, S.agv(k).v, q)];
end
set(H.txt, 'String', s, 'Color', 'k');
if out.stale, set(H.txt, 'Color', [0.8 0 0]); end
drawnow;
end
