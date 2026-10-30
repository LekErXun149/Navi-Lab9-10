function SC = phaseD_scenarios(cfg)
%PHASED_SCENARIOS  The Phase D test scenarios, built from the site in cfg.site.
%   SC = phaseD_scenarios(cfg)    cell array, one struct per scenario
%   Paths are defined relative to Lane 1, so they adapt to a new layout:
%     a = position along Lane 1 from its middle (m, + = driving direction)
%     b = sideways from the Lane 1 centre line (m, + = away from Lane 2)
%   Every scenario starts with a 3 s warm-up standing outside the fence and ends
%   with 5 s standing still.
%   Fields: id, name, group, P (true path), warm, agvLead, pass (function of the
%   metrics m), passText, and optional dropout, nlos, win, calmWin, holdWin/holdLane,
%   keepAgv/keepWin, feederAgv/feederWin, agv (AGV positions at fence entry).
%   S17 is the replay of the real walks (type 'replay').

S0 = safety_init(cfg); L1 = S0.lane(1); L2 = S0.lane(2);
c1 = (L1.a + L1.b) / 2; u = L1.u; n = L1.nrm;
c2 = (L2.a + L2.b) / 2;
if dot(c2 - c1, n) > 0, n = -n; end                   % n points away from Lane 2
pt  = @(a, b) c1 + a * u + b * n;
e   = cfg.site.laneWidth / 2;                         % lane edge (b)
Lh  = L1.L / 2;
o2  = -dot(c2 - c1, n);                               % Lane 2 centre is at b = -o2
bF1 = ray(c1, n, cfg.site.fence);                     % fence on the Lane 1 side (b)
bF2 = ray(c1, -n, cfg.site.fence);                    % fence on the Lane 2 side (-b)
out1 = bF1 + 1.0; out2 = -(bF2 + 1.0);                % 1 m outside the fence
Bt  = cfg.risk.bufBase + cfg.risk.bufK * 0.16;        % typical buffer
W = 0.8; SLOWW = 0.3; FAST = 1.5; warm = 3;
spMid = Lh;                                           % crossing point of a = 0 on Lane 1 (s)
SC = {};

% ---------------- Group 1: normal work - must not over-react ----------------
G = (cfg.site.fence - mean(cfg.site.fence, 1));
ring = mean(cfg.site.fence, 1) + G .* (1 + 1.0 ./ sqrt(sum(G.^2, 2)));
segs = {'wait', warm, [], ''};
for k = [2:size(ring, 1) 1], segs(end+1, :) = {'to', ring(k, :), W, ''}; end %#ok<AGROW>
SC{end+1} = mk('S1', 'Walk around outside the fence', 1, phaseD_path(ring(1, :), segs), warm, ...
    @(m) m.stopCmds == 0 && m.prod > 99, 'no STOP, AGVs at full speed (>99%)');

P = phaseD_path(pt(-Lh, out1), {'wait', warm, [], ''; 'to', pt(-Lh + 0.5, bF1 - 0.6), W, 'in'; ...
    'to', pt(Lh - 0.5, bF1 - 0.6), W, 'endpar'; 'to', pt(Lh - 0.5, out1), W, ''});
s = mk('S2', 'Walk inside the fence, parallel to Lane 1', 1, P, warm, ...
    @(m) ~m.stopInWin, 'no STOP while walking parallel (from 3 s after entering)');
s.calmWin = [P.mark.in + 3, P.mark.endpar]; SC{end+1} = s;

bMid = min(e + Bt + (cfg.risk.crawlBand + cfg.risk.slowBand) / 2, bF1 - 0.3);   % middle of the SLOW zone
P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, bMid), W, 'in'; 'wait', 30, [], 'stand'; ...
    'to', pt(0, out1), W, ''});
s = mk('S3', 'Stand still inside the fence, in the SLOW zone', 1, P, warm, ...
    @(m) ~m.stopInWin && m.flicker <= 1, 'no STOP while standing (from 3 s), <= 1 lane level change per minute');
s.win = [P.mark.stand + 3, P.mark.stand + 30]; s.calmWin = s.win; SC{end+1} = s;

% ---------------- Group 2: approaching a lane (E1) -------------------------
P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, e + 0.1), SLOWW, 'edge'; 'wait', 5, [], ''; ...
    'to', pt(0, out1), W, ''});
SC{end+1} = mk('S4', 'Slow approach (0.3 m/s) to the Lane 1 edge', 2, P, warm, ...
    @(m) m.nearMiss == 0 && m.stopCmds >= 1, 'no near miss, STOP issued');

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, 0), W, 'inlane'; 'wait', 3, [], ''; 'to', pt(0, out1), W, ''});
SC{end+1} = mk('S5', 'Normal walk (0.8 m/s) into Lane 1', 2, P, warm, ...
    @(m) m.nearMiss == 0, 'no near miss');

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, 0), FAST, 'inlane'; 'wait', 3, [], ''; 'to', pt(0, out1), W, ''});
SC{end+1} = mk('S6', 'Running (1.5 m/s) into Lane 1', 2, P, warm, ...
    @(m) m.nearMiss == 0, 'no near miss');

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, out2), W, ''});
SC{end+1} = mk('S7', 'Walk across both lanes', 2, P, warm, ...
    @(m) m.nearMiss == 0, 'no near miss');

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, e + 1.0), W, ''; 'wait', 2, [], ''; ...
    'to', pt(0, e + 2.0), 0.5, ''; 'wait', 2, [], ''; 'to', pt(0, 0), W, ''; 'wait', 2, [], ''; 'to', pt(0, out1), W, ''});
SC{end+1} = mk('S8', 'Hesitant: approach, back off, approach, enter', 2, P, warm, ...
    @(m) m.nearMiss == 0, 'no near miss');

% ---------------- Group 3: direction and lanes (E2) ------------------------
P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, 0), W, 'inlane'; 'wait', 2, [], ''; 'to', pt(0, out1), W, ''});
s = mk('S9', 'Cross Lane 1 just after an AGV has passed', 3, P, warm, ...
    @(m) m.nearMiss == 0 && m.kept, 'no near miss, the AGV that passed is never stopped');
s.agv = {'L1-A', spMid + cfg.agv.passMargin + 0.6; 'L1-B', spMid - 3.5};
tF = phaseD_tfence(P, cfg); s.keepAgv = 'L1-A';
s.keepWin = [tF, min(P.mark.inlane + 2, tF + (L1.L - s.agv{1, 2}) / cfg.agv.vMax - 0.2)]; SC{end+1} = s;

P = phaseD_path(pt(-1.5, out1), {'wait', warm, [], ''; 'to', pt(-1.5, 0), W, 'inlane'; ...
    'to', pt(1.5, 0), W, ''; 'to', pt(1.5, out1), W, ''});
s = mk('S10', 'Walk along inside Lane 1 (driving direction)', 3, P, warm, ...
    @(m) m.nearMiss == 0 && m.kept, 'no near miss, the AGV ahead is never stopped');
s.agv = {'L1-A', spMid - 1.5 + 3.0; 'L1-B', spMid - 1.5 - 3.5};
tF = phaseD_tfence(P, cfg); s.keepAgv = 'L1-A';
s.keepWin = [tF, min(P.mark.inlane + 1, tF + (L1.L - s.agv{1, 2}) / cfg.agv.vMax - 0.2)]; SC{end+1} = s;

P = phaseD_path(pt(-Lh + 0.4, out1), {'wait', warm, [], ''; 'to', pt(-Lh + 0.4, 0), W, 'inlane'; 'wait', 5, [], 'stand'; ...
    'to', pt(-Lh + 0.4, out1), W, ''});
s = mk('S11', 'Step into Lane 1 at its start (AGV coming round)', 3, P, warm, ...
    @(m) m.nearMiss == 0 && m.feeder, 'no near miss, the AGV coming round stays on the feeder track');
s.agv = {'L1-A', -cfg.agv.feedLen};
s.feederAgv = 'L1-A'; s.feederWin = [P.mark.inlane P.mark.stand + 5]; SC{end+1} = s;

% ---------------- Group 4: boundaries and stopping (E5) --------------------
bB = e + Bt + cfg.risk.crawlBand;                     % CRAWL zone edge (inside the fence)
P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, bB), W, ''; 'wait', 60, [], 'stand'; 'to', pt(0, out1), W, ''});
s = mk('S12', 'Stand exactly on the CRAWL boundary for 60 s', 4, P, warm, ...
    @(m) m.flicker <= 1, '<= 1 lane level change per minute (= 0.5 releases, as in Phase B)');
s.win = [P.mark.stand + 3 P.mark.stand + 60]; SC{end+1} = s;

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, 0), W, 'inlane'; 'wait', 20, [], ''; 'to', pt(0, out1), W, ''});
SC{end+1} = mk('S13', 'Stand in Lane 1 for 20 s, then leave', 4, P, warm, ...
    @(m) m.nearMiss == 0 && m.release <= 6, 'no near miss, AGVs released within 6 s of leaving the lane');

% ---------------- Group 5: bad data (E3, E4) -------------------------------
P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, e + 1.0), W, 'near'; 'wait', 8, [], ''; 'to', pt(0, out1), W, ''});
s = mk('S14', 'Signal lost for 3 s, 1 m from Lane 1', 5, P, warm, ...
    @(m) m.nearMiss == 0 && m.failsafe, 'no near miss, every AGV at least CRAWL while lost');
s.dropout = [P.mark.near + 2, P.mark.near + 5]; SC{end+1} = s;

P = phaseD_path(pt(0, out1 + 2), {'wait', 12, [], ''});
s = mk('S15', 'Signal lost for 3 s, outside the fence', 5, P, warm, ...
    @(m) m.failsafe && m.recover <= 3, 'CRAWL while lost, NORMAL within 3 s after');
s.dropout = [5 8]; SC{end+1} = s;

P = phaseD_path(pt(0, out1), {'wait', warm, [], ''; 'to', pt(0, e + 0.4), W, 'near'; 'wait', 10, [], ''; 'to', pt(0, out1), W, ''});
s = mk('S16', 'NLOS: one anchor reads +1 m for 3 s, next to Lane 1', 5, P, warm, ...
    @(m) m.nearMiss == 0 && m.held, 'no near miss, Lane 1 STOP held the whole time');
s.nlos = [1, P.mark.near + 3, P.mark.near + 6, 1.0];
s.holdLane = 1; s.holdWin = [P.mark.near + 1.5, P.mark.near + 9]; SC{end+1} = s;

% ---------------- Group 6: real data ---------------------------------------
s = mk('S17', 'Replay of the real walks 1 and 2 (26-Oct)', 6, [], 0, ...
    @(m) m.minClear >= 1.0, 'closest moving AGV >= 1.0 m from the estimated position');
s.type = 'replay'; s.files = {'walk1_1704', 'walk2'}; SC{end+1} = s;

% ---------------- Work shift (the productivity comparison) ----------------
w1 = pt(-2, e + 1.4); w2 = pt(2, e + 1.4); w3 = pt(2, -(o2 + e + 1.0));
P = phaseD_path(pt(-2, out1), {'wait', warm, [], ''; 'to', w1, W, ''; 'wait', 40, [], ''; 'to', w2, W, ''; 'wait', 30, [], ''; ...
    'to', w3, W, ''; 'wait', 40, [], ''; 'to', pt(2, out2), W, ''; 'wait', 30, [], ''; 'to', pt(0, out2), W, ''; ...
    'to', pt(0, out1), W, ''; 'wait', 20, [], ''});
SC{end+1} = mk('S18', 'Work shift: tasks near the lanes, crossings, breaks (~4 min)', 7, P, warm, ...
    @(m) m.nearMiss == 0, 'no near miss (productivity is compared with the basic system)');
end

% -------------------------------------------------------------------------
function s = mk(id, name, group, P, warm, pass, passText)
if ~isempty(P)                                        % 5 s standing at the end (time to release)
    tt = (P.t(end) + 0.04:0.04:P.t(end) + 5)';
    P.t = [P.t; tt]; P.xy = [P.xy; repmat(P.xy(end, :), numel(tt), 1)];
end
s = struct('id', id, 'name', name, 'group', group, 'type', 'sim', 'P', P, 'warm', warm, ...
           'agvLead', 1.0, 'pass', pass, 'passText', passText);
end

function t = ray(c, d, F)
% distance from c along unit direction d to the polygon F boundary
t = Inf; nF = size(F, 1);
for k = 1:nF
    q0 = F(k, :); q1 = F(mod(k, nF) + 1, :); e = q1 - q0;
    M = [d(:) -e(:)];
    if abs(det(M)) < 1e-12, continue; end
    x = M \ (q0(:) - c(:));
    if x(1) > 0 && x(2) >= 0 && x(2) <= 1, t = min(t, x(1)); end
end
end
