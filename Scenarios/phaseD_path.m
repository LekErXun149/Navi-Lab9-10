function P = phaseD_path(p0, segs, dt)
%PHASED_PATH  Scripted true path of the person, sampled every dt seconds.
%   P = phaseD_path(p0, segs, dt)
%     p0    start point [x y]
%     segs  cell array, one row per step, always 4 columns:
%             {'to',   [x y],   speed, 'name'}  walk straight to a point (m/s);
%                                              'name' (or '') remembers the arrival time
%             {'wait', seconds, [],    'name'}  stand still; 'name' remembers the start time
%   P.t (Nx1), P.xy (Nx2), P.mark.<name> = time (s)
if nargin < 3, dt = 0.04; end
T = 0; X = p0; mark = struct();
for k = 1:size(segs, 1)
    s = segs(k, :);
    switch s{1}
        case 'to'
            q = s{2}; d = norm(q - X(end, :));
            if ~isempty(s{4}), mark.(s{4}) = T(end) + d / s{3}; end
            if d > 0, T(end+1) = T(end) + d / s{3}; X(end+1, :) = q; end %#ok<AGROW>
        case 'wait'
            if ~isempty(s{4}), mark.(s{4}) = T(end); end
            T(end+1) = T(end) + s{2}; X(end+1, :) = X(end, :); %#ok<AGROW>
    end
end
P.t = (0:dt:T(end))';
P.xy = [interp1(T, X(:, 1), P.t) interp1(T, X(:, 2), P.t)];
P.mark = mark;
end
