function kf = uwb_kf_init(cfg)
%UWB_KF_INIT  Empty constant-velocity Kalman filter. State x = [px py vx vy]'.
%   The filter starts itself on the first good position fix.
kf.cfg        = cfg.kf;
kf.x          = nan(4, 1);
kf.P          = nan(4);
kf.t          = NaN;     % time of current state (s)
kf.tLastGood  = NaN;     % time of last accepted measurement (s)
kf.nReject    = 0;       % consecutive gated-out measurements
kf.started    = false;
end
