%% Clear past plots, variables and console prints
close all; clear all; clc;

% Load data 
load 'EE5114_CA1.mat';

% acx = x-axis accelerometer reading
% acy = y-axis accelerometer reading
% acz = z-axis accelerometer reading
% 
% phi = Roll angle computed by the drone's on-board computer
% tht = Pitch angle computed by the drone's on-board computer
% psi = Yaw angle computed by the drone's on-board computer 
% 
% fix = GPS position fix signal 
% eph = GPS horizontal variance 
% epv = GPS vertical variance 
% lat = GPS Latitude
% lon = GPS Longitude
% alt = GPS altitude
% gps_nSat = Number of GPS satellites
% 
% out1 = Motor 1 signal
% out2 = Motor 2 signal
% out3 = Motor 3 signal
% out4 = Motor 4 signal

%% Accelerometer plot
figure; set(gcf,'numbertitle','off','name','Acceleration');  
subplot(3,1,1); plot(t, acx, 'b'); ylim([-2 2]); ylabel('acx (m/s^2)'); grid on; 
subplot(3,1,2); plot(t, acy, 'b'); ylim([-2 2]); ylabel('acy (m/s^2)'); grid on; 
subplot(3,1,3); plot(t, acz, 'b'); ylabel('acz (m/s^2)'); xlabel('time (s)'); grid on; 

%% Euler angles plot
figure; set(gcf,'numbertitle','off','name','Euler Angles');  
subplot(3,1,1); plot(t, rad2deg(phi), 'b'); ylabel('Roll (degree)'); grid on; 
subplot(3,1,2); plot(t, rad2deg(tht), 'b'); ylabel('Pitch (degree)'); grid on; 
subplot(3,1,3); plot(t, rad2deg(psi), 'b'); ylabel('Yaw (degree)'); xlabel('time (s)'); grid on; 

%% GPS plot
figure; set(gcf,'numbertitle','off','name','GPS');  
subplot(3,2,1); plot(t, lon); ylabel('Longitude'); grid on;
subplot(3,2,3); plot(t, lat); ylabel('Latitude'); grid on;
subplot(3,2,5); plot(t, alt); ylabel('Altitude'); grid on; xlabel('time (s)');

subplot(3,2,2); plot(t, gps_nSat, '.'); ylabel('Sat'); grid on;
subplot(3,2,4); plot(t, eph); ylabel('Eph'); grid on; ylim([0 5]);
subplot(3,2,6); plot(t, epv); ylabel('Epv'); grid on; ylim([0 5]); xlabel('time (s)');

%% Motor signal plot
figure; set(gcf,'numbertitle','off','name','Motor Signal');  
hold on;
plot(t,out1,'r');
plot(t,out2,'g');
plot(t,out3,'b');
plot(t,out4,'y');
legend('Motor1','Motor2','Motor3','Motor4'); 
ylabel('Motor inputs'); xlabel('time (s)'); ylim([1000 2000]); grid on;

%% Convert GPS raw measurements to local NED position values

% Identifying Valid GPS Signals
start_row = -1;

% Find the row where both gps_nSat >= 4 and fix == 3
for i = 1:length(gps_nSat)
    if gps_nSat(i) >= 4 && fix(i) == 3
        start_row = i;
        break; % Exit after finding the first matching row
    end
end

% If no matching row is found
if start_row == -1
    disp('GPS positioning ability and accuracy are low');
end

% Trim all relevant variables
t = t(start_row:end);
lat = lat(start_row:end);
lon = lon(start_row:end);
alt = alt(start_row:end);
acx = acx(start_row:end);
acy = acy(start_row:end);
acz = acz(start_row:end);

% Convert GPS raw measurements to local NED position values
% Define reference point (initial latitude, longitude, altitude)
lat0 = deg2rad(lat(1));  % Convert reference latitude to radians
lon0 = deg2rad(lon(1));  % Convert reference longitude to radians
alt0 = alt(1);  % Reference altitude in meters

% Earth's ellipsoid parameters for WGS-84
a = 6378137;  % Earth's equatorial radius (semi-major axis) in meters
b = 6356752;  % Earth's polar radius (semi-minor axis) in meters
e2 = 1 - (b^2 / a^2);  % First eccentricity squared

% Convert the current latitude, longitude to radians
lat_rad = deg2rad(lat);
lon_rad = deg2rad(lon);
% Prime vertical radius of curvature at reference point
N0 = a / sqrt(1 - e2 * sin(lat0)^2);  

% Convert reference point (lat0, lon0, alt0) to ECEF
X_e0 = (N0 + alt0) * cos(lat0) * cos(lon0);
Y_e0 = (N0 + alt0) * cos(lat0) * sin(lon0);
Z_e0 = (b^2 / a^2 * N0 + alt0) * sin(lat0);

% Initialize NED coordinate arrays
x_ned = zeros(size(lat));  % North (m)
y_ned = zeros(size(lat));  % East (m)
z_ned = zeros(size(lat));  % Down (m)

% Loop through each GPS point and convert it to NED
for i = 1:length(lat)
    % Compute prime vertical radius of curvature at current latitude
    N = a / sqrt(1 - e2 * sin(lat_rad(i))^2);
    
    % Convert current GPS (lat, lon, alt) to ECEF coordinates
    X_e = (N + alt(i)) * cos(lat_rad(i)) * cos(lon_rad(i));
    Y_e = (N + alt(i)) * cos(lat_rad(i)) * sin(lon_rad(i));
    Z_e = (b^2 / a^2 * N + alt(i)) * sin(lat_rad(i));
    
    % Compute the difference in ECEF coordinates
    delta_X = X_e - X_e0;
    delta_Y = Y_e - Y_e0;
    delta_Z = Z_e - Z_e0;
     
    % Apply rotation matrix to convert from ECEF to NED
    % Rotation matrix based on reference point's latitude and longitude
    R = [-sin(lat0) * cos(lon0), -sin(lat0) * sin(lon0),  cos(lat0);
         -sin(lon0),              cos(lon0),             0;
         -cos(lat0) * cos(lon0), -cos(lat0) * sin(lon0), -sin(lat0)];
    
    % Apply rotation to get NED coordinates
    ned_coords = R * [delta_X; delta_Y; delta_Z];
    
    x_ned(i) = ned_coords(1);  % North
    y_ned(i) = ned_coords(2);  % East
    z_ned(i) = ned_coords(3);  % Down
end

% Plot the GPS to NED positions
figure; set(gcf,'numbertitle','off','name','GPS to NED Position');  
subplot(3,1,1); plot(t, x_ned, 'b'); ylabel('x_{NED} (m)'); grid on;
subplot(3,1,2); plot(t, y_ned, 'b'); ylabel('y_{NED} (m)'); grid on;
subplot(3,1,3); plot(t, z_ned, 'b'); ylabel('z_{NED} (m)'); xlabel('time (s)'); grid on;


%% Implement EKF to estimate NED position and velocity
% Define the second takeoff and landing times based on the data
second_takeoff_time = 1439.76; % Replace with actual second takeoff time
second_landing_time = 1659.7;  % Replace with actual second landing time

t_min = second_takeoff_time - 10 * 60; % 10 minutes before 2nd takeoff
t_max = second_landing_time + 5 * 60;  % 5 minutes after 2nd landing

% Find indices for t_min and t_max
idx_min = find(t >= t_min, 1);
idx_max = find(t <= t_max, 1, 'last');

% Trim data to the selected time range
t = t(idx_min:idx_max);
acx = acx(idx_min:idx_max);
acy = acy(idx_min:idx_max);
acz = acz(idx_min:idx_max);
x_ned = x_ned(idx_min:idx_max);
y_ned = y_ned(idx_min:idx_max);
z_ned = z_ned(idx_min:idx_max);
phi = phi(idx_min:idx_max);
tht = tht(idx_min:idx_max);
psi = psi(idx_min:idx_max);


% Initial State: 
%[x_ned, y_ned, z_ned, vx_ned, vy_ned, vz_ned, bias_ax, bias_ay, bias_az]
x_est = zeros(9, length(t));  % Estimated state over time
x_est(:, 1) = [x_ned(1); y_ned(1); z_ned(1); 0; 0; 0; 0; 0; 0];
% Initialize state (position from GPS, velocity and bias set to zero)

% Process noise covariance (Q) and measurement noise covariance (R)
P = diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 1, 1, 1]) ; % Initial covariance matrix
Q = diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.5, 0.5, 0.5]);  % Process noise
R = diag([0.8, 0.8, 0.8]);  % Measurement noise

% Precompute the rotation matrix function
R_gb = @(phi, tht, psi) [cos(tht)*cos(psi), cos(tht)*sin(psi), -sin(tht);
                        sin(phi)*sin(tht)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(tht)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(tht);
                        cos(phi)*sin(tht)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(tht)*sin(psi)-sin(phi)*cos(psi), cos(phi)*cos(tht)];

% Initialize the gps_available array
gps_available = false(length(t), 1);

% Check GPS data availability
for k = 2:length(t)
     % If the current GPS data differs from the previous time step, consider GPS data updated
    gps_available(k) = (lat(k) ~= lat(k-1)) || (lon(k) ~= lon(k-1)) || (alt(k) ~= alt(k-1));
end
gps_available(1) = true;

% EKF Implementation
for k = 2:length(t)
    dt = t(k) - t(k-1);
   
    % Get the current rotation matrix from body to global frame
    R_g_b = R_gb(phi(k), tht(k), psi(k));

    % 1. Prediction Step
    % F matrix with rotation matrix R_g_b incorporated
    F = [1, 0, 0, dt, 0, 0, -dt^2/2, 0, 0;
        0, 1, 0, 0, dt, 0, 0, -dt^2/2, 0;
        0, 0, 1, 0, 0, dt, 0, 0, -dt^2/2;
        0, 0, 0, 1, 0, 0, -dt, 0, 0;
        0, 0, 0, 0, 1, 0, 0, -dt, 0;
        0, 0, 0, 0, 0, 1, 0, 0, -dt;
        0, 0, 0, 0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1];
    %F(7:9, 7:9) = R_g_b;

    G = [dt^2/2, 0, 0;
         0, dt^2/2, 0;
         0, 0, dt^2/2;
         dt, 0, 0;
         0, dt, 0;
         0, 0, dt;
         0, 0, 0;
         0, 0, 0;
         0, 0, 0];
    
    % Get the current rotation matrix from body to global frame
    % R_g_b = R_gb(phi(k), tht(k), psi(k));

    % Compute the control input: accelerometer readings in global frame
    g = -9.81;   % Gravity constant
    u_k = R_g_b * [acx(k); acy(k); acz(k)-g];  % Subtract gravity in the z-axis

    % State prediction
    x_pred = F * x_est(:, k-1) + G * u_k;

    % Covariance prediction
    P_pred = F * P * F' + Q;

    % Check if GPS data is available
    if gps_available(k)
        % 2. Correction Step 
        H_k = [eye(3), zeros(3, 6)];

        % Modify the measurement vector z_k
        z_k = [x_ned(k); y_ned(k); z_ned(k)];

        % Kalman gain
        K = P_pred * H_k' / (H_k * P_pred * H_k' + R);

        % Update state estimate
        x_est(:, k) = x_pred + K * (z_k - H_k * x_pred);

        % Update the covariance matrix
        P = (eye(9) - K * H_k) * P_pred;

    else
        % If no GPS data is available, only perform prediction, no correction
        x_est(:, k) = x_pred;
        P = P_pred;
    end

end

%% Result plots

% Plot EKF estimated positions (NED)
figure; 
set(gcf,'numbertitle','off','name','EKF Estimated NED Position');

subplot(3,1,1); 
plot(t, x_est(1,:), 'r');
ylabel('x_{NED} (m)', 'FontSize', 12); 
grid on;
%title('EKF Estimated NED Positions', 'FontSize', 14); % Title for the entire figure

subplot(3,1,2); 
plot(t, x_est(2,:), 'r'); 
ylabel('y_{NED} (m)', 'FontSize', 12); 
grid on;

subplot(3,1,3); 
plot(t, x_est(3,:), 'r'); 
ylabel('z_{NED} (m)', 'FontSize', 12); 
xlabel('time (s)', 'FontSize', 12); 
grid on;

% Plot EKF estimated velocities (NED)
figure; 
set(gcf,'numbertitle','off','name','EKF Estimated NED Velocity');

subplot(3,1,1); 
plot(t, x_est(4,:), 'b'); 
ylabel('v_x (m/s)', 'FontSize', 12); 
grid on;
%title('EKF Estimated NED Velocities', 'FontSize', 14); % Title for the entire figure

subplot(3,1,2); 
plot(t, x_est(5,:), 'b'); 
ylabel('v_y (m/s)', 'FontSize', 12); 
grid on;

subplot(3,1,3); 
plot(t, x_est(6,:), 'b');
ylabel('v_z (m/s)', 'FontSize', 12); 
xlabel('time (s)', 'FontSize', 12); 
grid on;

% Plot EKF estimated accelerometer biases
figure; 
set(gcf,'numbertitle','off','name','EKF Estimated Accelerometer Bias');

subplot(3,1,1); 
plot(t, x_est(7,:), 'b', 'LineWidth', 0.5); 
ylabel('bias_{ax} (m/s^2)', 'FontSize', 12); 
grid on;
%title('EKF Estimated Accelerometer Biases', 'FontSize', 14); % Title for the entire figure

subplot(3,1,2); 
plot(t, x_est(8,:), 'b', 'LineWidth', 0.5); 
ylabel('bias_{ay} (m/s^2)', 'FontSize', 12); 
grid on;

subplot(3,1,3); 
plot(t, x_est(9,:), 'b', 'LineWidth', 0.5); 
ylabel('bias_{az} (m/s^2)', 'FontSize', 12); 
xlabel('time (s)', 'FontSize', 12); 
grid on;


