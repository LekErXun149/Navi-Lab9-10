function S = safety_init(cfg)
%SAFETY_INIT  Initial state of the AGV safety system (lanes, AGVs, warning memories).
%   S = safety_init(cfg)
%   AGVs start spread evenly along each lane at normal speed.

nL = numel(cfg.site.lanes);
for l = 1:nL
    a = cfg.site.lanes(l).p0; b = cfg.site.lanes(l).p1;
    S.lane(l).a = a; S.lane(l).b = b;
    S.lane(l).L = norm(b - a);
    S.lane(l).u = (b - a) / S.lane(l).L;                 % driving direction
    S.lane(l).nrm = [-S.lane(l).u(2) S.lane(l).u(1)];    % sideways unit vector
    S.lane(l).mem = {[], [], [], []};                    % hysteresis memories (E5): stop-dist, stop-TTL, slow-dist, slow-TTL
end

k = 0;
for l = 1:nL
    for j = 1:cfg.agv.perLane
        k = k + 1;
        S.agv(k).lane    = l;
        S.agv(k).name    = sprintf('L%d-%c', l, 'A' + j - 1);
        S.agv(k).s       = S.lane(l).L * (j - 0.5) / cfg.agv.perLane;   % position along lane (m)
        S.agv(k).v       = cfg.agv.vMax;                                 % speed (m/s)
        S.agv(k).desired = 1;                                            % level decided now
        S.agv(k).applied = 1;                                            % level the AGV is acting on
        S.agv(k).queue   = [-Inf 1];                                     % [time level] commands (latency)
    end
end

S.memFence = [];
S.est = [];               % latest position estimate
S.tsLastFrame = -Inf;     % sim time when the last frame arrived
S.ts = NaN;               % sim time of the last step
end
