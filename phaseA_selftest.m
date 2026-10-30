%% phaseA_selftest.m - quick checks that the Phase A building blocks work.
% Run this first on any new laptop. All lines should say PASS.
clear; clc;
cfg = uwb_config();
T = cell(0, 2);   % {test name, pass?}

%% 1. Decoder: example frame from the LinkTrack manual (section 8.1.2.5)
hex = ['55 04 ac 00 02 01 ba 66 1d 00 06 09 ff de 0a 00 df ff ff e8 03 00 fa ff ff 1a 00 00 00 00 00 c0 12 00 00 00 00 00 00 00 ' ...
       '27 ac e2 3c 56 ed 1c 3c d2 70 3b bd 32 57 66 3e 3b cb 1b 41 93 70 61 bd 25 b2 6b 41 a1 22 6c 41 da da 6b 41 6d 23 e9 23 ' ...
       '70 dd db f7 30 3f 5f d6 31 3f ba 81 1e 3e 47 69 e2 bd 91 9b 40 40 c5 23 00 00 40 40 5c d5 1c 00 00 00 1d 00 6d 13 04 01 ' ...
       '00 6b 0c 00 b1 9f a6 66 1d 00 16 45 01 01 ac 15 00 b4 a1 a6 66 1d 00 d3 01 01 02 48 1a 00 ca 9f a6 66 1d 00 38 ba 01 03 ' ...
       '2b 12 00 c6 a0 a6 66 1d 00 64 40 25'];
frame = uint8(hex2dec(strsplit(strtrim(hex), ' ')))';
% surround with junk and split across two reads to test re-sync and buffering
stream = [uint8([1 2 85 3]) frame frame(1:50)];
[F, rest, st] = nf2_parse(stream, 4);
[F2, ~, st2] = nf2_parse([rest frame(51:end)], 4);
ok = st.good == 1 && st2.good == 1;                            T(end+1,:) = {'decoder: finds frames in a noisy, split stream', ok};
ok = abs(F.t - 1926.842) < 1e-9;                               T(end+1,:) = {'decoder: system_time = 1926842 ms', ok};
ok = max(abs(F.posNA - [2.782 -0.033 1.000])) < 1e-9;          T(end+1,:) = {'decoder: pos = (2.782, -0.033, 1.000) m', ok};
ok = max(abs(F.D - [3.179 5.548 6.728 4.651])) < 1e-9;         T(end+1,:) = {'decoder: ranges A0..A3 match manual', ok};
ok = F.fp(1) == -88.5 && F.rx(1) == -79.5;                     T(end+1,:) = {'decoder: fp/rx RSSI of A0 = -88.5 / -79.5 dB', ok};
ok = abs(F.voltage - 4.973) < 1e-9 && F.nNodes == 4;           T(end+1,:) = {'decoder: voltage 4.973 V, 4 nodes', ok};
bad = frame; bad(30) = bad(30) + 1;
[~, ~, st3] = nf2_parse(bad, 4);
ok = st3.good == 0 && st3.badChecksum == 1;                    T(end+1,:) = {'decoder: rejects a corrupted frame (checksum)', ok};

%% 2. Trilateration with height correction (noise-free synthetic)
pTrue = [5.0 2.5];
d = sqrt(sum((cfg.anchor.xy - pTrue).^2, 2) + (cfg.anchor.h(:) - cfg.tag.h).^2)';
[p, info] = uwb_trilat(d, cfg);
ok = info.ok && norm(p - pTrue) < 1e-3;                        T(end+1,:) = {'trilat: 4 anchors, exact ranges -> exact position', ok};
d3 = d; d3(2) = NaN;
[p, info] = uwb_trilat(d3, cfg);
ok = info.ok && info.nUsed == 3 && norm(p - pTrue) < 1e-3;     T(end+1,:) = {'trilat: works with one anchor missing', ok};
d2 = d; d2([2 3]) = NaN;
[~, info] = uwb_trilat(d2, cfg);
ok = ~info.ok;                                                 T(end+1,:) = {'trilat: refuses a fix with only 2 anchors', ok};
dBad = d; dBad(1) = dBad(1) + 1.0;
[~, info] = uwb_trilat(dBad, cfg);
ok = info.resRMS > 0.2;                                        T(end+1,:) = {'trilat: 1 m range error -> large residual (E3b)', ok};

%% 3. Kalman filter: straight walk at 0.8 m/s with 15 cm noise
rng(1, 'twister');                 % modern generator (legacy rand('seed') breaks rng later)
dt = 0.04; t = (0:dt:20)'; v = [0.8 0];
P = [1 2.5] + t * v;
Z = P + 0.15 * randn(size(P));
kf = uwb_kf_init(cfg); sp = nan(size(t)); pe = nan(size(t)); sg = nan(size(t));
for k = 1:numel(t)
    [kf, est] = uwb_kf_step(kf, t(k), Z(k, :), cfg.kf.sigma0);
    sp(k) = est.speed; pe(k) = norm(est.pos - P(k, :)); sg(k) = est.sigmaPos;
end
s = t > 3;
ok = abs(mean(sp(s)) - 0.8) < 0.05;                            T(end+1,:) = {sprintf('KF: mean speed %.2f m/s (true 0.80)', mean(sp(s))), ok};
ok = sqrt(mean(pe(s).^2)) < 0.15;                              T(end+1,:) = {sprintf('KF: position RMSE %.3f m < raw noise 0.15 m', sqrt(mean(pe(s).^2))), ok};
[kf, est] = uwb_kf_step(kf, t(end) + 0.04, [NaN NaN], NaN);
[kf, est2] = uwb_kf_step(kf, t(end) + 0.60, [NaN NaN], NaN);
ok = est2.sigmaPos > est.sigmaPos && est2.age > cfg.health.staleTime;  T(end+1,:) = {'KF: no data -> uncertainty grows, data marked stale', ok};
[kf, est] = uwb_kf_step(kf, t(end) + 0.64, P(end, :) + [3 0], cfg.kf.sigma0);
ok = ~est.accepted;                                            T(end+1,:) = {'KF: 3 m jump is rejected by the gate', ok};

%% 4. Hysteresis + dwell (E5, used by the risk logic)
tt = (0:0.04:4)'; c = tt >= 0.5 & tt < 1.0;            % warning condition true 0.5-1.0 s
mem = []; on = false(size(tt));
for j = 1:numel(tt), [on(j), mem] = risk_hyst(c(j), tt(j), ~c(j), 1.0, mem); end
ok = on(find(tt >= 0.5, 1)) && on(find(tt >= 1.9, 1)) && ~on(find(tt >= 2.1, 1));
T(end+1,:) = {'hysteresis: ON at once, OFF only after 1 s dwell', ok};
mem = []; flips = 0; prev = false; d = 0.2 * sin(2*pi*3*tt);   % jitter +-0.2 m around a boundary
for j = 1:numel(tt), [o, mem] = risk_hyst(d(j) < 0, tt(j), d(j) > 0.45, 1.0, mem); flips = flips + (prev && ~o); prev = o; end
T(end+1,:) = {'hysteresis: no flicker with +-0.2 m jitter (gap 0.45 m)', flips == 0};

for k = 1:size(T, 1)
    if T{k, 2}, res = 'PASS'; else, res = '*** FAIL ***'; end
    fprintf('%-55s %s\n', T{k, 1}, res);
end
fprintf('\n%d PASS, %d FAIL\n', sum([T{:, 2}]), sum(~[T{:, 2}]));
