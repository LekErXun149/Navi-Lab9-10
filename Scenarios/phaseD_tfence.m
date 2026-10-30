function t = phaseD_tfence(P, cfg)
%PHASED_TFENCE  First time the TRUE path is inside the fence (Inf if never).
t = Inf;
in = inpolygon(P.xy(:, 1), P.xy(:, 2), cfg.site.fence(:, 1), cfg.site.fence(:, 2));
k = find(in, 1);
if ~isempty(k), t = P.t(k); end
end
