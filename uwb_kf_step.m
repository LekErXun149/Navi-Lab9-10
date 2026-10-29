function [kf, est] = uwb_kf_step(kf, t, z, sigmaMeas)
%UWB_KF_STEP  One Kalman predict + (optional) update.
%   [kf, est] = uwb_kf_step(kf, t, z, sigmaMeas)
%     t         frame time (s)
%     z         measured position [x y], or [NaN NaN] if no valid fix
%     sigmaMeas measurement std (m) for this fix (E3b: grows with residual)
%   est fields: pos, vel, speed, heading (deg, 0 = +x, CCW), sigmaPos (m,
%   worst-direction 1-sigma, E3a), P (4x4), accepted, reinit, age (s since good update)

c = kf.cfg;
est = struct('pos', [NaN NaN], 'vel', [NaN NaN], 'speed', NaN, 'heading', NaN, ...
             'sigmaPos', NaN, 'P', nan(4), 'accepted', false, 'reinit', false, 'age', Inf);
haveZ = all(isfinite(z));

% ---- (re)initialise on first fix, after a long gap, time reset, or too many rejections
dt = t - kf.t;
if ~kf.started || ~isfinite(dt) || dt <= 0 || dt > c.maxGap || kf.nReject >= c.maxReject
    if haveZ
        kf.x = [z(:); 0; 0];
        kf.P = diag([c.P0pos c.P0pos c.P0vel c.P0vel]);
        kf.t = t; kf.tLastGood = t; kf.nReject = 0; kf.started = true;
        est = fill(est, kf, t); est.accepted = true; est.reinit = true;
    else
        kf.started = false;
    end
    return
end

% ---- predict
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
q = c.q;
Q = q * [dt^3/3 0 dt^2/2 0; 0 dt^3/3 0 dt^2/2; dt^2/2 0 dt 0; 0 dt^2/2 0 dt];
kf.x = F * kf.x;
kf.P = F * kf.P * F' + Q;
kf.t = t;

% ---- update (with innovation gate)
if haveZ
    H = [1 0 0 0; 0 1 0 0];
    R = sigmaMeas^2 * eye(2);
    y = z(:) - H * kf.x;
    S = H * kf.P * H' + R;
    d2 = y' / S * y;                       % Mahalanobis distance^2
    if d2 <= c.gateChi2
        K = kf.P * H' / S;
        kf.x = kf.x + K * y;
        kf.P = (eye(4) - K * H) * kf.P;
        kf.P = (kf.P + kf.P') / 2;
        kf.tLastGood = t; kf.nReject = 0;
        est.accepted = true;
    else
        kf.nReject = kf.nReject + 1;
    end
end
est = fill(est, kf, t);
end

function est = fill(est, kf, t)
est.pos      = kf.x(1:2)';
est.vel      = kf.x(3:4)';
est.speed    = hypot(kf.x(3), kf.x(4));
est.heading  = atan2d(kf.x(4), kf.x(3));
est.sigmaPos = sqrt(max(eig(kf.P(1:2, 1:2))));
est.P        = kf.P;
est.age      = t - kf.tLastGood;
end
