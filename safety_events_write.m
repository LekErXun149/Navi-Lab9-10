function files = safety_events_write(EV, base, cfg)
%SAFETY_EVENTS_WRITE  Write the event log to two CSV files (open in Excel).
%   files = safety_events_write(EV, base, cfg)
%     <base>_events.csv  one row per event (see safety_events); times from the run start
%     <base>_visits.csv  one row per stay inside the fence
%   A visit still open at the end of the run is closed with exit_s = end time.
if ~isempty(EV.visit)                                   % close an open visit
    v = EV.visit;
    EV.vis(end+1, :) = [v.t0, EV.tLast, EV.tLast - v.t0, v.worst, v.stops, v.dMin, v.tLane - v.tWarn];
end
t0 = EV.t0;                                             % times relative to the run start
d = fileparts(base);
if ~isempty(d) && ~exist(d, 'dir'), mkdir(d); end
files = {[base '_events.csv'], [base '_visits.csv']};

fid = fopen(files{1}, 'w');
fprintf(fid, ['time_s,event,agv,from,to,reason,person_x_m,person_y_m,person_speed_mps,' ...
              'lane_dist_m,ttl_s,latency_s,settle_s,stop_margin_m,buffer_m\n']);
for i = 1:size(EV.ev, 1)
    r = EV.ev(i, :);
    fprintf(fid, '%.2f,%s,%s,%s,%s,"%s",%s,%s,%s,%s,%s,%s,%s,%s,%s\n', r{1} - t0, r{2}, r{3}, r{4}, r{5}, ...
        strrep(r{6}, '"', ''''), num(r{7}), num(r{8}), num(r{9}), num(r{10}), num(r{11}), ...
        num(r{12}), num(r{13}), num(r{14}), num(r{15}));
end
fclose(fid);

nm = cfg.agv.names;
fid = fopen(files{2}, 'w');
fprintf(fid, 'visit,entry_s,exit_s,duration_s,worst_level,stop_commands,closest_moving_agv_m,warning_before_lane_s\n');
for i = 1:size(EV.vis, 1)
    v = EV.vis(i, :);
    fprintf(fid, '%d,%.2f,%.2f,%.2f,%s,%d,%s,%s\n', i, v(1) - t0, v(2) - t0, v(3), nm{v(4)}, v(5), num(v(6)), num(v(7)));
end
fclose(fid);
end

function s = num(x)
if isempty(x) || ~isfinite(x), s = ''; else, s = sprintf('%.2f', x); end
end
