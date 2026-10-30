function S = safety_init(cfg)
%SAFETY_INIT  Initial state of the AGV safety system (lanes, AGVs, warning memories).
%   S = safety_init(cfg)
%   AGVs start spread evenly along lane + feeder track, standing still in the
%   fail-safe level: they only move off once the safety system has valid data.
%   s = position along the lane from p0; s < 0 means on the feeder track before p0.

nL = numel(cfg.site.lanes);
for l = 1:nL
    a = cfg.site.lanes(l).p0; b = cfg.site.lanes(l).p1;
    S.lane(l).a = a; S.lane(l).b = b;
    S.lane(l).L = norm(b - a);
    S.lane(l).u = (b - a) / S.lane(l).L;                 % driving direction
    S.lane(l).nrm = [-S.lane(l).u(2) S.lane(l).u(1)];    % sideways unit vector
    S.lane(l).mem = {[], [], [], [], []};                % hysteresis memories (E5): stop-dist, stop-TTL, slow-dist, slow-TTL, entry
end

k = 0;
for l = 1:nL
    for j = 1:cfg.agv.perLane
        k = k + 1;
        S.agv(k).lane    = l;
        S.agv(k).name    = sprintf('L%d-%c', l, 'A' + j - 1);
        Lloop = S.lane(l).L + cfg.agv.feedLen;                           % lane + feeder track
        S.agv(k).s       = Lloop * (j - 0.5) / cfg.agv.perLane - cfg.agv.feedLen;  % position (m); < 0 = on feeder
        S.agv(k).v       = 0;                                            % speed (m/s): held at start-up
        S.agv(k).desired = cfg.health.staleLevel;                        % level decided now
        S.agv(k).applied = cfg.health.staleLevel;                        % level the AGV is acting on
        S.agv(k).queue   = [-Inf cfg.health.staleLevel];                 % [time level] commands (latency)
    end
end

S.memFence = [];
S.fenceWasOn = false; S.tFenceOn = -Inf;   % entry rule
S.est = [];               % latest position estimate
S.tsLastFrame = -Inf;     % sim time when the last frame arrived
S.ts = NaN;               % sim time of the last step
end
