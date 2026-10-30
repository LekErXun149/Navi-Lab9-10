function [d, s, q, n] = geo_seg(p, a, b)
%GEO_SEG  Distance from point p to segment a-b.
%   [d, s, q, n] = geo_seg(p, a, b)
%     d  distance (m)
%     s  position of the closest point along the segment, 0 at a ... |b-a| at b
%     q  closest point [x y]
%     n  unit vector from q towards p ([0 0] if p is on the segment)
ab = b - a; L = norm(ab);
u  = ab / L;
s  = min(max(dot(p - a, u), 0), L);
q  = a + s * u;
r  = p - q; d = norm(r);
if d > 1e-9, n = r / d; else, n = [0 0]; end
end
