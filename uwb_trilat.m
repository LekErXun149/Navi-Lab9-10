function [p, info] = uwb_trilat(d, cfg, p0)
%UWB_TRILAT  2D position from UWB ranges, with height correction.
%   [p, info] = uwb_trilat(d, cfg)       d = 1xN slant ranges (m), NaN = missing
%   [p, info] = uwb_trilat(d, cfg, p0)   p0 = starting guess (e.g. last position)
%
%   1) Height correction: r_i = sqrt(d_i^2 - (h_anchor_i - h_tag)^2)
%   2) Linear least squares (subtract first equation) -> initial guess
%   3) Gauss-Newton nonlinear least squares on the horizontal ranges
%
%   info.ok       true if >= cfg.tri.minAnchors ranges and the solve converged
%   info.nUsed    number of anchors used
%   info.res      per-anchor residual (predicted - measured horizontal range), NaN if unused
%   info.resRMS   RMS residual (m)  -> E3b: large value = ranges disagree = bad fix

A  = cfg.anchor.xy;
dh = cfg.anchor.h(:) - cfg.tag.h;
d  = d(:);
use = isfinite(d) & d > 0;
info = struct('ok', false, 'nUsed', nnz(use), 'res', nan(size(d)), 'resRMS', NaN);
p = [NaN NaN];
if nnz(use) < cfg.tri.minAnchors, return; end

r  = sqrt(max(d(use).^2 - dh(use).^2, cfg.tri.minHoriz^2));   % horizontal ranges
Au = A(use, :);

% --- linear least squares seed
if nargin >= 3 && all(isfinite(p0))
    x = p0(:);
else
    M = 2 * (Au(2:end, :) - Au(1, :));
    b = r(1)^2 - r(2:end).^2 + sum(Au(2:end, :).^2, 2) - sum(Au(1, :).^2);
    x = M \ b;
end

% --- Gauss-Newton
for it = 1:cfg.tri.maxIter
    diffv = x' - Au;                      % rows: (p - a_i)
    rho   = sqrt(sum(diffv.^2, 2));
    rho   = max(rho, 1e-6);
    J     = diffv ./ rho;                 % d rho / d p
    e     = rho - r;
    dx    = -(J' * J) \ (J' * e);
    if any(~isfinite(dx)), return; end
    x = x + dx;
    if norm(dx) < 1e-4, break; end
end

rho = sqrt(sum((x' - Au).^2, 2));
res = rho - r;
info.res(use) = res;
info.resRMS   = sqrt(mean(res.^2));
info.ok       = all(isfinite(x));
p = x';
end
