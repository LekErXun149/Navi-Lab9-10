function cfg = phaseD_site(cfg, clearance)
%PHASED_SITE  Replace the fence by a rectangle around the lanes with a given clearance.
%   cfg = phaseD_site(cfg, clearance)   clearance (m) = fence to the nearest lane edge,
%   on all sides (lane ends too). [] keeps cfg.site.fence from uwb_config.m.
%   The rectangle is aligned with Lane 1. Anchors are unchanged, so positions far
%   outside the anchor area get the real (worse) accuracy.
if isempty(clearance), return; end
S = safety_init(cfg); u = S.lane(1).u; n = S.lane(1).nrm; o = S.lane(1).a;
pts = [];
for l = 1:numel(S.lane), pts = [pts; S.lane(l).a; S.lane(l).b]; end %#ok<AGROW>
al = (pts - o) * u'; ac = (pts - o) * n';
w = cfg.site.laneWidth / 2 + clearance;
a0 = min(al) - clearance; a1 = max(al) + clearance; b0 = min(ac) - w; b1 = max(ac) + w;
C = [a0 b0; a0 b1; a1 b1; a1 b0];
cfg.site.fence = o + C(:, 1) * u + C(:, 2) * n;
end
