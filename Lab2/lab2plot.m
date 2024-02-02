%% Data plotting for ECE410/557 lab 2.
% Last modified August 25, 2023
clear all;
close all;
clc;

%% @students 
% TODO: Ensure the following serial port is set to the same serial port 
% being used by the Arduino controller.
port = 'COM4';

% You may wish to change the total time of the experiment.
stop_time = 20; % in seconds

%% !!-- DO NOT MODIFY anything below --!!
serial = serialport(port, 115200); % open the serial port for reading
serial_delim = ','; % delimeter used in serial port communications

T_plot = 0.03; % interval at which Arduino writes to the serial port
n_samples = fix(stop_time/T_plot);
time = T_plot*(0:n_samples - 1);

% preallocate arrays for the measured and desired positions
z_encoder = zeros(1, n_samples);
z_desired = zeros(1, n_samples);

[plot_handle, ~, figure_handle] = make_plot(stop_time, 0.05);
fprintf("Now collecting data from experiment, please wait %d seconds.\n", stop_time);

for i=2:n_samples
    try
        serial_values = read_delimited_serial(serial, serial_delim);
    catch e
        warning("Serial read failed, data collection loop terminating early.");
        break;
    end

    z_encoder(i) = serial_values(1); 
    z_desired(i) = serial_values(2);
    show_plot(plot_handle, figure_handle, time(1:i), z_encoder(1:i));
end
fprintf("Data collection completed.\n");

%% Clean up the serial object
try
    clear serial;
catch e  % in case serial was already cleared
end

%% Plot all outstanding data
figure;
hold on;
plot(time, z_desired, 'r', 'Linewidth',2);
plot(time, z_encoder, 'b', 'Marker','.','Linewidth',1);

xlabel('Time [s]', 'FontWeight','bold', 'FontSize',14)
ylabel('Position [m]', 'FontWeight','bold', 'FontSize',14);
legend('Measured Position', 'Desired Position');

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