classdef serialport < handle
%SERIALPORT  FAKE serialport for testing UwbSource without hardware.
%   Streams test_fake_serial/walk1.bin at ~25 frames/s in random-sized chunks.
    properties
        data; pos = 0; t0; bytesPerSec = 4300;
    end
    properties (Dependent)
        NumBytesAvailable
    end
    methods
        function obj = serialport(port, baud) %#ok<INUSD>
            fid = fopen('test_fake_serial/walk1.bin'); obj.data = fread(fid, Inf, 'uint8=>uint8')'; fclose(fid);
            obj.t0 = tic;
        end
        function n = get.NumBytesAvailable(obj)
            target = min(numel(obj.data), round(toc(obj.t0) * obj.bytesPerSec * 20));   % 20x speed
            n = max(0, target - obj.pos);
            if n > 0, n = randi(n); end                                               % random chunking
        end
        function b = read(obj, n, type) %#ok<INUSD>
            b = obj.data(obj.pos+1 : obj.pos+n); obj.pos = obj.pos + n;
        end
        function flush(obj) %#ok<MANU>
        end
    end
end
