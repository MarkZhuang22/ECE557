% 4.2 Symbolic linearization and controllability analysis
M = 1.1911;
m = 0.2300;
l = 0.3302;
g = 9.8;
alpha_1 = 1.7265;
alpha_2 = 10.5;

syms z z_dot theta theta_dot u real;
syms x z_ddot theta_ddot f;

z_ddot_n = (-m*l*sin(theta)*theta_dot^2 + m*g*sin(theta)*cos(theta) + alpha_1*u - alpha_2*z_dot);
z_ddot_d = M + m*(sin(theta))^2;
z_ddot = z_ddot_n / z_ddot_d;

theta_ddot_n = -m*l*sin(theta)*cos(theta)*theta_dot^2 + (M+m)*g*sin(theta) + (alpha_1*u-alpha_2*z_dot)*cos(theta);
theta_ddot_d = l*(M + m*(sin(theta))^2);
theta_ddot = theta_ddot_n / theta_ddot_d;

x=[z ; z_dot ; theta ; theta_dot];
f=[z_dot ; z_ddot ; theta_dot ; theta_ddot];
matlabFunction(eval(f),'Vars',{x,u},'File','cartpend','Outputs',{'xdot'});
result = cartpend([0 0 pi/2 0]',1);   %%Different from lab manual
disp("Cartpend =");
disp(result.')

x_bar = zeros(4,1);
u_bar = 0;
A = subs(jacobian(f,x),[x;u],[x_bar;u_bar]);
B = subs(jacobian(f,u),[x;u],[x_bar;u_bar]);
controllabilityMatrix = ctrb(double(A), double(B));
rankValue = rank(controllabilityMatrix);
disp(rankValue);
disp("A =");
disp(A);
disp("B =");
disp(B);

% 4.3 Eigenvalue Assignment
p_acker = [-5, -5, -5, -5];
K_acker = acker(double(A), double(B), p_acker);
e_val_acker = eig(double(A)+double(B)*(-K_acker));
disp("e_val_acker =");
disp(e_val_acker.');

p_place = [-5, -5.1, -4.9, -5.2];
K_place = place(double(A), double(B), p_place);
e_val_place = eig(double(A)+double(B)*(-K_place));
disp("e_val_place =");
disp(e_val_place.');

% 4.4 Linear Quadratic Optimal Control
q1 = 2;
q2 = 1;
R = 0.2;
Q = [q1 0 0 0; 0 0 0 0; 0 0 q2 0; 0 0 0 0];
[K_LQR1,~,~] = lqr(double(A), double(B), Q, R);
e_val_LQR1 = eig(double(A)+double(B)*(-K_LQR1));
disp("e_val_LQR1 =");
disp(e_val_LQR1.');

q1 = 0.1;
q2 = 1;
R = 1;
Q = [q1 0 0 0; 0 0 0 0; 0 0 q2 0; 0 0 0 0];
[K_LQR2,~,~] = lqr(double(A), double(B), Q, R);
e_val_LQR2 = eig(double(A)+double(B)*(-K_LQR2));
disp("e_val_LQR2 =");
disp(e_val_LQR2.');

% 4.5 Simulation of Linear Controllers and Linearized Plant
x0=2*[0; 0; pi/24; 0]; 
K = K_place;
out_place=sim('lab3_linear.slx',10);

K = K_LQR1;
out_LQR1=sim('lab3_linear.slx',10);

K = K_LQR2;
out_LQR2=sim('lab3_linear.slx',10);

figure(1)
subplot(311)
title('Comparison of the three controllers with linearized plant')
%subtitle('Preparation')   % use subtitle in R2020
ylabel('z')
hold on
plot(out_place.z)
plot(out_LQR1.z)
plot(out_LQR2.z)
subplot(312)
ylabel('theta')
hold on
plot(out_place.theta)
plot(out_LQR1.theta)
plot(out_LQR2.theta)
subplot(313)
ylabel('u')
hold on
plot(out_place.u)
plot(out_LQR1.u)
plot(out_LQR2.u)
legend('pole assignment','LQR1','LQR2','Location','NorthEast')



