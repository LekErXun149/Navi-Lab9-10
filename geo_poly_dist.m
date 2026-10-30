function [inside, d] = geo_poly_dist(p, P)
%GEO_POLY_DIST  Is p inside polygon P (corners in order, Nx2), and its distance to the edge.
%   [inside, d] = geo_poly_dist(p, P)
inside = inpolygon(p(1), p(2), P(:, 1), P(:, 2));
d = Inf;
n = size(P, 1);
for k = 1:n
    d = min(d, geo_seg(p, P(k, :), P(mod(k, n) + 1, :)));
end
end
