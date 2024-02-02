% Data plotting for ECE410/557 lab 4.
% Last modified August 25, 2023
clear all;
close all;
clc;

%% @students 
% TODO: Ensure the following serial port is set to the same serial port 
% being used by the Arduino controller.
port = 'COM4'; 

% You may wish to change the total time of the experiment.
stop_time = 60; % in seconds

%% !!-- DO NOT MODIFY anything below --!!
serial = serialport(port, 115200); % open the serial port for reading
serial_delim = ','; % delimeter used in serial port communications

T_plot = 0.03; % interval at which Arduino writes to the serial port
n_samples = fix(stop_time/T_plot);
time = T_plot*(0:n_samples - 1);

% preallocate arrays for the measured and desired positions
z_encoder = zeros(1, n_samples);
z_desired = zeros(1, n_samples);
theta_encoder = zeros(1, n_samples);
zdot_observer = zeros(1, n_samples);
thetadot_observer = zeros(1, n_samples);
control_input = zeros(1, n_samples);

[plot_handle, ~, figure_handle] = make_plot(stop_time, 0.05);
fprintf("Now collecting data from experiment, please wait %d seconds.\n", stop_time);

for i=1:n_samples
    try
        serial_values = read_delimited_serial(serial, serial_delim);
    catch e
        warning("Serial read failed, data collection loop terminating early.");
        break;
    end

    % Square wave
    z_desired(i) = serial_values(3);

    % Encoder measurements
    z_encoder(i) = serial_values(1);
    theta_encoder(i) = serial_values(2);

    % Observer states
    zdot_observer(i) = serial_values(4);
    thetadot_observer(i) = serial_values(5);
    
    % Control input
    control_input(i) = serial_values(6);
    
    show_plot(plot_handle, figure_handle, time(1:i), z_encoder(1:i));
end
fprintf("Data collection completed.\n");

%% Clean up the serial object
try
    clear serial;
catch e  % in case serial was already cleared
end

%% Time derivative and spline interpolation
% For comparison to observer
zdot_spline = spline_velocity(time, z_encoder, 10);
thetadot_spline = spline_velocity(time, theta_encoder, 5);

%% Plot all outstanding data
figure
hold on
plot(time, z_desired, 'r', 'Linewidth', 2, 'DisplayName', 'Position');
plot(time, z_encoder, 'b', 'Marker', '.', 'Linewidth', 1, 'DisplayName', 'Reference Position');
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14)
ylabel('Position [m]', 'FontWeight','bold', 'FontSize', 14);
title('Cart Position as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
legend;

% pendulum angle
figure;
plot(time, theta_encoder);
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Angle [rad]', 'FontWeight','bold', 'FontSize',14);
title('Pendulum Angle as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
grid on;

% cart speed
figure;
hold on;
plot(time, zdot_observer, 'DisplayName', 'Cart velocity observer');
plot(time, zdot_spline, 'DisplayName', 'Cart velocity (spline)')
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Speed [m/s]', 'FontWeight','bold', 'FontSize',14);
title('Cart Velocity as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
legend;
grid on;

% pendulum speed
figure;
hold on;
plot(time, thetadot_observer, 'DisplayName', 'Pendulum angular velocity observer')
plot(time, thetadot_spline, 'DisplayName', 'Pendulum angular velocity (spline)');
xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
ylabel('Pendulum Speed [rad/s]', 'FontWeight','bold', 'FontSize',14);
title('Pendulum Angular Velocity as a Function of Time', ...
    'FontWeight','bold', 'FontSize',15);
legend;
grid on;


%% Functions to read serial data and plot 
function [plot_handle, axes_handle, figure_handle] = make_plot(xlim_val, ylim_val)
    figure_handle = figure('NumberTitle','off', 'Name','Tracking');
    axes_handle = axes('Parent',figure_handle, 'YGrid','on', 'XGrid','on');
    plot_handle = plot(axes_handle, 0,0, 'Marker','.', 'LineWidth',1);
    xlim(axes_handle, [0, xlim_val]);
    ylim(axes_handle, [-ylim_val, ylim_val]);
    
    xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14);
    ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);
    title('Cart Position as a Function of Time', ...
        'FontWeight','bold', 'FontSize',15);
end

function [] = show_plot(plot_handle, figure_handle, xdata, ydata)
    set(plot_handle, 'YData', ydata, 'XData', xdata);
    set(figure_handle, 'Visible','on');
    drawnow;
end

function [values] = read_delimited_serial(serial_object, delim)
    serial_data = strip(readline(serial_object)); 
    parsed_string = split(serial_data, delim);
    values = str2double(parsed_string);
end

function [velocity] = spline_velocity(time, position, filter_order)
    % Obtain cart velocity from cart position encoder;
    position_pp = spline(time, position);
    velocity_pp = fnder(position_pp);
    velocity = ppval(velocity_pp, time);
    
    % Smooth velocity data using an aggressive Butterworth LP filter
    [b,a] = butter(filter_order, 0.25);
    velocity = filtfilt(b, a, velocity);
end
