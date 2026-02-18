% Connect to Arduino
s = serialport("COM9", 115200);  
configureTerminator(s, "LF");
flush(s);

% Log data for 10 seconds
duration = 10;
data = [];
tic;
while toc < duration
    line = readline(s);
    vals = str2double(split(line, ","));
    if numel(vals) == 5
        data = [data; vals'];
    end
end

% Parse columns
t         = data(:,1) / 1000;  % ms to seconds
pitch     = data(:,2);
roll      = data(:,3);
pitchCmd  = data(:,4);
rollCmd   = data(:,5);

save('gimbal_data.mat', 't', 'pitch', 'roll', 'pitchCmd', 'rollCmd');
disp('Data saved!');