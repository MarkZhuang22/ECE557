% Constants
alpha1 = 1.7265; alpha2 = 10.5; M = 0.8211;

% State-space matrices
A = [0, 1; 0, -alpha2/M];
B = [0; alpha1/M];
C = [1 0];

%% 4.2 State feedback stabilization
% State feedback design
[K1, K2] = state_feedback_design(A, B, [-5 -5]);
fprintf("K1 = %g, K2 = %g\n", K1, K2);% K1 = -11.8897, K2 = 1.3258
K = [K1 K2]; % This K will be used in lab2_part1.slx

% Check eigenvalues of A + BK
eigenvalues = eig(A + B * K);

% Display the eigenvalues
disp('Eigenvalues of A + BK are:');% Eigenvalues of A + BK are: [-5,-5]
disp(eigenvalues);

%% 4.3 Simulink simulation of state feedback controller

% Settling time for lab2_part1:
% zd jumps from -0.03 to 0.03        at t = 5.000,10.000,15.000 seconds
% z reaches (0.03 - 0.06 * 0.02) = 0.0288 at t = 6.168 seconds
% Settling time = 6.168 - 5.000 = 1.168 seconds

%% 4.4 State estimation and output feedback control
% The desired eigenvalues are -10, -10
[L1, L2] = observer_design(A, [-10, -10]);
L = [L1; L2];  % Observer gain matrix
A_minus_LC = A - L * C;

fprintf("L1 = %g, L2 = %g\n", L1, L2);  % L1 = 7.21228, L2 = 7.7714
% Check the eigenvalues of A - LC
eigenvalues = eig(A_minus_LC);

% Display the eigenvalues for verification
disp('Eigenvalues of A - LC are:'); % Eigenvalues of A - LC are:[-10,-10]
disp(eigenvalues);

% Test output_feedback_controller function
[Actrl, Bctrl, Cctrl, Dctrl] = ...
    output_feedback_controller(A, B, C, [-10, -10], [-20, -20]);

% Display the matrices for verification
disp('Actrl = ');
disp(Actrl);
disp('Bctrl = ');
disp(Bctrl);
disp('Cctrl = ');
disp(Cctrl);
disp('Dctrl = ');
disp(Dctrl);

% Function definition
function [K1, K2] = state_feedback_design(A, B, p)
%  State feedback design, function to find K1, K2 based on A, B, and 
% eigenvalues in p
    K1 = -prod(p) / B(2);
    K2 = (-A(2, 2) + sum(p)) / B(2);
end

function [L1, L2] = observer_design(A, p)
% Extract the value of a22 from the A matrix for calculations
    a22 = A(2, 2);
% Calculate L1 and L2 based on the desired eigenvalues and a22
    L1 = a22 - sum(p);
    L2 = L1 * a22 + prod(p);
end

function [Actrl, Bctrl, Cctrl, Dctrl] = ...
    output_feedback_controller(A, B, C, p_feedback, p_observer)
    % Call state_feedback_design to get K
    [K1, K2] = state_feedback_design(A, B, p_feedback);
    K = [K1, K2];
    
    % Call observer_design to get L
    [L1, L2] = observer_design(A, p_observer);
    L = [L1; L2];

    % Calculate controller matrices
    Actrl = A + B * K - L * C;
    Bctrl = [L, -B * K];
    Cctrl = K;
    Dctrl = [0, -K];
end

