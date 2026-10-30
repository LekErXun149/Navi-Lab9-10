function cfg = phaseD_policy(cfg, version)
%PHASED_POLICY  Set cfg.policy for one version of the safety system.
%   'full'  this project (E1-E5 on)      'basic' stop every AGV whenever anyone is inside the fence
%   'noE1'  no time-to-lane prediction   'noE2'  whole lane stops (no direction rule)
%   'noE3'  fixed buffer                 'noE5'  no hysteresis / dwell
%   The basic system uses the same localisation, fence hysteresis and fail-safe,
%   so the comparison only measures the decision rules.
p = struct('mode', 'full', 'E1', true, 'E2', true, 'E3', true, 'E5', true, 'fixedBuf', 0.72);
if isfield(cfg, 'policy') && isfield(cfg.policy, 'fixedBuf'), p.fixedBuf = cfg.policy.fixedBuf; end
switch version
    case 'full'
    case 'basic', p.mode = 'basic';
    case 'noE1',  p.E1 = false;
    case 'noE2',  p.E2 = false;
    case 'noE3',  p.E3 = false;
    case 'noE5',  p.E5 = false;
    otherwise, error('unknown version %s', version);
end
cfg.policy = p;
end
