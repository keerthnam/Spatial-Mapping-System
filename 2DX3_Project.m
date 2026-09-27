
clear; clc;

%% Configuration 
port = "COM3";    
baudrate = 115200;    % matches MCU UART_Init()
depth    = 3;         % number of scans (Deliverable 2 = 3)
measurements = 32;    % readings per scan (360 / 11.25 = 32)

%% Open serial port
device = serialport(port, baudrate);
device.Timeout = 30;  % 30s timeout - enough time to press button and scan
configureTerminator(device, "CR/LF");
fprintf("Opened port: %s\n", port);
flush(device);

%% Wait for user ready
input("Press Enter in MATLAB, then press PJ0 (SW1) on the board to start scan 1...");

%% Constants for coordinate conversion
angle_increment = 360 / measurements;  % 11.25 degrees per step
X = [];
Y = [];
Z = [];
scan_number = 1;
dataMatrix = [];

%% Receive all measurement data
fprintf("\nReceiving %d measurements (%d scans x %d readings)...\n", ...
        depth*measurements, depth, measurements);

for i = 1:depth*measurements

    % Notify user when to press button for scans 2 and 3
    if i == measurements + 1
        fprintf("\n--- Scan 1 complete. Move sensor if needed, then press PJ0 for scan 2 ---\n");
    elseif i == 2*measurements + 1
        fprintf("\n--- Scan 2 complete. Move sensor if needed, then press PJ0 for scan 3 ---\n");
    end

    data = readline(device);
    fprintf("  [%d/%d] %s\n", i, depth*measurements, data);

    if isempty(data)
        continue;
    end

    % Parse the 5 comma-separated values from MCU
    point = sscanf(data, '%f, %f, %f, %f, %f');

    if numel(point) == 5
        dataMatrix(end+1, :) = point;

        % Angle for this measurement within its scan
        angle = angle_increment * mod(i - 1, measurements);

        % Distance is column 2 (RangeStatus=1, Distance=2, ...)
        distance = dataMatrix(end, 2);

        % Increment scan number every 32 measurements
        if mod(i, measurements) == 0
            scan_number = scan_number + 1;
        end

        % Convert polar -> Cartesian
        % X = depth axis (scan number * 100mm = 10cm apart)
        % Y = horizontal distance
        % Z = vertical distance
        X(i) = scan_number * 100;
        Y(i) = distance * cosd(angle);
        Z(i) = distance * sind(angle);
    end
end

fprintf("\nAll %d measurements received.\n", depth*measurements);

%% Save raw data to CSV
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
filename = sprintf('scan_data_%s.csv', timestamp);
writematrix(dataMatrix, filename);
fprintf("Raw data saved to: %s\n", filename);

%% Close serial port
clear device;
fprintf("Serial port closed.\n");

%% 3D Visualization
figure('Name', 'Student 400579618 - 2DX3 Spatial Map', 'NumberTitle', 'off');

% Scatter plot of all points
scatter3(X, Y, Z, 20, 'filled');
hold on;

pointsPerSet = measurements;  % 32
numSets = floor(length(X) / pointsPerSet);

% Connect points within each scan ring (closed loop)
for setIdx = 1:numSets
    startIndex = (setIdx - 1) * pointsPerSet + 1;
    endIndex   = startIndex + pointsPerSet - 1;

    % Draw ring outline
    plot3(X(startIndex:endIndex), Y(startIndex:endIndex), Z(startIndex:endIndex), ...
          'b-o', 'MarkerSize', 2, 'MarkerFaceColor', 'b', 'LineWidth', 1);

    % Close the ring (last point back to first)
    plot3([X(endIndex), X(startIndex)], ...
          [Y(endIndex), Y(startIndex)], ...
          [Z(endIndex), Z(startIndex)], ...
          'b-o', 'MarkerSize', 2, 'MarkerFaceColor', 'b');
end

% Connect corresponding points between scans (vertical lines)
for pointIdx = 1:pointsPerSet
    for setIdx = 1:numSets-1
        p1 = (setIdx - 1) * pointsPerSet + pointIdx;
        p2 = p1 + pointsPerSet;
        plot3([X(p1), X(p2)], [Y(p1), Y(p2)], [Z(p1), Z(p2)], ...
              'k-', 'LineWidth', 0.5);
    end
end


hold off;
title('Student 400579618 - 2DX3 Deliverable 2 - 3D Spatial Map');
xlabel('X - Depth (mm)');
ylabel('Y - Width (mm)');
zlabel('Z - Height (mm)');
grid on;
axis equal;
view(35, 25);

fprintf("Plot complete.\n");
