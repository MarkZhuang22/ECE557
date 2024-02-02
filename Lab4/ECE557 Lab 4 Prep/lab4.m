% Clear all previous variables and figures
clear all

% Define physical parameters of the system
M = 1.1911; % Mass of the cart
m = 0.2300; % Mass of the pendulum
l = 0.3302; % Length of the pendulum
g = 9.8;    % Acceleration due to gravity
alpha_1 = 1.7265; % Motor force constant
alpha_2 = 10.5;   % Friction coefficient
Ulim = 5.875;    % Actuator saturation limit

% Define symbolic variables for the state and control input
syms z z_dot theta theta_dot u real;

% Equations of motion for the cart-pendulum system
z_ddot = (-m*l*sin(theta)*theta_dot^2 + m*g*sin(theta)*cos(theta) + alpha_1*u - alpha_2*z_dot) / (M + m*(sin(theta))^2);
theta_ddot = (-m*l*sin(theta)*cos(theta)*theta_dot^2 + (M+m)*g*sin(theta) + (alpha_1*u-alpha_2*z_dot)*cos(theta))/ (l*(M + m*(sin(theta))^2));

% Define the state vector and its derivative
x = [z ; z_dot ; theta ; theta_dot];
f = [z_dot ; z_ddot ; theta_dot ; theta_ddot];

% Generate MATLAB function for system dynamics
matlabFunction(eval(f),'Vars',{x,u},'File','cartpend','Outputs',{'xdot'});

% Compute linearization of the system
x_bar = zeros(4,1); % Equilibrium point
u_bar = 0;          % Equilibrium control input
A = double(subs(jacobian(f,x),[x;u],[x_bar;u_bar]));
B = double(subs(jacobian(f,u),[x;u],[x_bar;u_bar]));
C = [1 0 0 0; 0 0 1 0]; % Output matrix

% Check controllability and observability
controllabilityMatrix = ctrb(A, B);
rankValue = rank(controllabilityMatrix); 
disp(rankValue); % Display rank of controllability matrix
O = obsv(A,C);
observability = rank(O);
disp(observability); % Display rank of observability matrix

% Pole placement for state feedback controller
p_K = [-5, -5.1, -4.9, -5.2];
K = place(A, B, p_K);
e_val_K = eig(A+B*(-K));
disp("e_val_K =");
disp(e_val_K.');

% LQR controller design
q1 = 2;
q2 = 1;
R = 0.2;
Q = [q1 0 0 0; 0 0 0 0; 0 0 q2 0; 0 0 0 0];
[K_LQR1,~,~] = lqr(A, B, Q, R);
e_val_LQR1 = eig(A+B*(-K_LQR1));
disp("e_val_LQR1 =");
disp(e_val_LQR1.');

% Observer design using pole placement
p_L = [-10, -10.01, -9.99, -10.02];
%p_L = [-40, -40.01, -39.99, -40.02];
L = place(A', C', p_L)';
e_val_L = eig(A - (L)*C);
disp("e_val_L =");
disp(e_val_L.');

% Define augmented control system for integral control
Actrl = A - B*K - L*C;
Bctrl = [L B*K];
Cctrl = -K;
Dctrl = [zeros(1,2) K];
x0 = [0; 0; pi/24; 0]; % Initial condition

% Simulate and plot the system response
out=sim('lab4_prep.slx',30);
figure(1)
subplot(311)
plot(out.z.Time, out.z.Data);
title('Cart Position z(t)');
ylabel('z');
xlabel('Time (s)');
hold on;

subplot(312)
plot(out.theta.Time, out.theta.Data);
title('Pendulum Angle ?(t)');
ylabel('theta');
xlabel('Time (s)');
hold on;

subplot(313)
plot(out.u.Time, out.u.Data);
title('Control Input u(t)');
ylabel('u');
xlabel('Time (s)');
hold on;

% Check for control input saturation
u = out.u.Data;
time = out.u.Time;
saturation_limit = 5.875;  

% Find the indices where control input is at or beyond saturation limits
saturation_indices = find(abs(u) >= saturation_limit);

% Check the duration of saturation
if ~isempty(saturation_indices)
    saturation_times = time(saturation_indices);
    % Determine the continuous stretches of time where u is saturated
else
    disp('No saturation detected.');
end