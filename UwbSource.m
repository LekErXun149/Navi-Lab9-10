classdef UwbSource < handle
%UWBSOURCE  One interface for live tag data and recorded replays.
%   src = UwbSource('serial', cfg)                 live tag on cfg.serial.port
%   src = UwbSource('replay', cfg, file)           replay at real-time speed
%   src = UwbSource('replay', cfg, file, speed)    speed = 2 -> 2x, Inf -> all at once
%
%   F = src.poll()   returns all NEW frames since the last call (frame batch)
%   src.isDone()     true when a replay has finished
%   src.close()      release the serial port / close the raw log
%
%   Because both modes return the same frame batch, the application code is
%   identical for live tracking and for replaying a recording.

    properties
        mode            % 'serial' or 'replay'
        cfg
        nAnch
        % serial
        sp = [];        % serialport object
        buf = uint8([]);
        rawFid = -1;
        rawFile = '';
        stats = struct('good', 0, 'badChecksum', 0, 'skipped', 0);
        % replay
        data            % frame batch
        idx = 0;
        tic0
        t0
        speed = 1;
    end

    methods
        function obj = UwbSource(mode, cfg, file, speed)
            obj.mode = lower(mode); obj.cfg = cfg; obj.nAnch = cfg.anchor.n;
            switch obj.mode
                case 'serial'
                    obj.sp = serialport(cfg.serial.port, cfg.serial.baud);  % MATLAB R2019b+
                    flush(obj.sp);
                    if ~isempty(char(cfg.serial.rawLogDir))
                        if ~exist(cfg.serial.rawLogDir, 'dir'), mkdir(cfg.serial.rawLogDir); end
                        obj.rawFile = fullfile(char(cfg.serial.rawLogDir), ...
                            ['raw_' datestr(now, 'yyyymmdd_HHMMSS') '.bin']);
                        obj.rawFid = fopen(obj.rawFile, 'w');
                    end
                case 'replay'
                    obj.data = nf2_load(file, obj.nAnch);
                    if nargin >= 4, obj.speed = speed; end
                    obj.tic0 = tic; obj.t0 = obj.data.t(1);
                otherwise
                    error('mode must be ''serial'' or ''replay''');
            end
        end

        function F = poll(obj)
            if strcmp(obj.mode, 'serial')
                nb = obj.sp.NumBytesAvailable;
                if nb > 0
                    newb = read(obj.sp, nb, 'uint8');
                    if obj.rawFid > 0, fwrite(obj.rawFid, newb, 'uint8'); end
                    obj.buf = [obj.buf uint8(newb(:)')];
                end
                [F, obj.buf, st] = nf2_parse(obj.buf, obj.nAnch);
                obj.stats.good = obj.stats.good + st.good;
                obj.stats.badChecksum = obj.stats.badChecksum + st.badChecksum;
                obj.stats.skipped = obj.stats.skipped + st.skipped;
            else
                n = numel(obj.data.t);
                if isinf(obj.speed)
                    last = min(n, obj.idx + 250);           % chunks of 10 s
                else
                    tNow = obj.t0 + toc(obj.tic0) * obj.speed;
                    last = obj.idx;
                    while last < n
                        tn = obj.data.t(last+1);
                        if last >= 1
                            jump = tn - obj.data.t(last);
                            if jump < 0 || jump > 1    % time reset or long gap in file: skip the wait
                                obj.t0 = tn; obj.tic0 = tic; tNow = tn;
                            end
                        end
                        if tn > tNow, break; end
                        last = last + 1;
                    end
                end
                F = nf2_slice(obj.data, obj.idx+1:last);
                obj.idx = last;
            end
        end

        function tf = isDone(obj)
            tf = strcmp(obj.mode, 'replay') && obj.idx >= numel(obj.data.t);
        end

        function close(obj)
            if obj.rawFid > 0, fclose(obj.rawFid); obj.rawFid = -1; end
            obj.sp = [];          % releases the COM port
        end

        function delete(obj)
            obj.close();
        end
    end
end
