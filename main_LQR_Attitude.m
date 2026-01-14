%% =========================================================
%  LQR Attitude Control for Rigid Spacecraft (Quaternion Form)
%  Full Version: Quaternion Error + Angular Velocity + Control Torque
% =========================================================
clear; clc; close all;

%% -----------------------------
% Global Plot Style (Unified)
% -----------------------------
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultLineLineWidth',2);
set(groot,'defaultAxesLineWidth',1);
set(groot,'defaultAxesGridLineStyle','--');
set(groot,'defaultAxesXGrid','on');
set(groot,'defaultAxesYGrid','on');
set(groot,'defaultAxesBox','on');

% Unified Color Scheme (per axis/component)
c_q0 = [0.85 0.00 0.00];   % Red
c_q1 = [0.00 0.00 0.85];   % Blue
c_q2 = [0.85 0.45 0.00];   % Orange
c_q3 = [0.00 0.60 0.00];   % Green

c_w1 = c_q0;   % wx  -> red
c_w2 = c_q1;   % wy  -> blue
c_w3 = c_q2;   % wz  -> orange

c_u1 = c_q0;   % ux  -> red
c_u2 = c_q1;   % uy  -> blue
c_u3 = c_q2;   % uz  -> orange

%% -----------------------------
% 1. Spacecraft Parameters
% -----------------------------
MOI = [84.7089, 84.7089, 169.4178];   % Moment of inertia [kg·m^2]
I = diag(MOI);

%% -----------------------------
% 2. Initial Conditions
% -----------------------------
q0 = [sqrt(1-0.3^2*3); 0.3; -0.3; -0.3];    % initial quaternion
w0 = [0; 0; 0];      % initial angular velocity
x0 = [q0; w0];

%% -----------------------------
% 3. Target Attitude
% -----------------------------
qt = [1; 0; 0; 0];                  % target quaternion
qt = qt / norm(qt);                     % normalize
wt = [0; 0; 0];

%% -----------------------------
% 4. Linearized Error Model (LQR)
% -----------------------------
A = [zeros(3), 0.5*eye(3);
     zeros(3), zeros(3)];

B = [zeros(3);
     diag(1 ./ MOI)];

Q = diag([50 50 50  10 10 10]);   % attitude > angular velocity
R = diag([1 1 1]);                  % control effort

K = lqr(A, B, Q, R);

disp('------------------------------------')
disp('LQR Gain Matrix K:')
disp(K)
disp('------------------------------------')

%% -----------------------------
% 5. Simulation Parameters
% -----------------------------
dt = 0.01;
T  = 120;
tspan = 0:dt:T;

x = zeros(7, length(tspan));   % [q0 q1 q2 q3 w1 w2 w3]
u = zeros(3, length(tspan));
x(:,1) = x0;

%% -----------------------------
% 6. Nonlinear Closed-Loop Simulation
% -----------------------------
for k = 1:length(tspan)-1

    q = x(1:4,k);
    w = x(5:7,k);

    % Quaternion vector error
    q_err_vec = q(2:4) - qt(2:4);
    w_err = w - wt;

    
    x_err = [q_err_vec; w_err];

    % LQR control law
    u(:,k) = -K * x_err;

    % Integrate nonlinear dynamics
    x(:,k+1) = x(:,k) + dt * attitude_dynamics(x(:,k), u(:,k), I);

    % Normalize quaternion
    x(1:4,k+1) = x(1:4,k+1) / norm(x(1:4,k+1));
end

%% -----------------------------
% 7. Quaternion Error Metrics
% -----------------------------
q_err_hist = zeros(4, length(tspan));
theta_err = zeros(1, length(tspan));

for k = 1:length(tspan)
    q_err_hist(:,k) = x(1:4,k) - qt;
    theta_err(k) = 2 * acos(abs(dot(x(1:4,k), qt)));
end
theta_err_deg = theta_err * 180/pi;

%% -----------------------------
% 8. Plot Results
% -----------------------------

% 8.1 Quaternion
figure;
subplot(2,1,1)
plot(tspan, x(1,:), 'Color', c_q0); hold on
plot(tspan, x(2,:), 'Color', c_q1)
plot(tspan, x(3,:), 'Color', c_q2)
plot(tspan, x(4,:), 'Color', c_q3)
xlabel('Time [s]','Interpreter','latex')
ylabel('Quaternion','Interpreter','latex')
legend('$q_0$','$q_1$','$q_2$','$q_3$','Interpreter','latex')
title('Attitude Quaternion Response','Interpreter','latex')

% 8.2 Angular velocity
subplot(2,1,2)
plot(tspan, x(5,:), 'Color', c_w1); hold on
plot(tspan, x(6,:), 'Color', c_w2)
plot(tspan, x(7,:), 'Color', c_w3)
xlabel('Time [s]','Interpreter','latex')
ylabel('Angular Velocity [rad/s]','Interpreter','latex')
legend('$\omega_x$','$\omega_y$','$\omega_z$','Interpreter','latex')
title('Angular Velocity Response','Interpreter','latex')

% 8.3 Control torque
figure;
plot(tspan, u(1,:), 'Color', c_u1); hold on
plot(tspan, u(2,:), 'Color', c_u2)
plot(tspan, u(3,:), 'Color', c_u3)
xlabel('Time [s]','Interpreter','latex')
ylabel('Control Torque [N$\cdot$m]','Interpreter','latex')
legend('$u_x$','$u_y$','$u_z$','Interpreter','latex')
title('LQR Control Input','Interpreter','latex')

% 8.4 Quaternion error components (All in one plot)
figure;
plot(tspan, q_err_hist(1,:), 'Color', c_q0, 'LineWidth', 2); hold on
plot(tspan, q_err_hist(2,:), 'Color', c_q1, 'LineWidth', 2)
plot(tspan, q_err_hist(3,:), 'Color', c_q2, 'LineWidth', 2)
plot(tspan, q_err_hist(4,:), 'Color', c_q3, 'LineWidth', 2)
xlabel('Time [s]','Interpreter','latex')
ylabel('Quaternion Error','Interpreter','latex')
legend('$e_{q0}$','$e_{q1}$','$e_{q2}$','$e_{q3}$','Interpreter','latex')
title('Quaternion Error Components','Interpreter','latex')

% 8.5 Rotation angle error
figure;
plot(tspan, theta_err_deg, 'Color', c_q0, 'LineWidth', 2)
xlabel('Time [s]','Interpreter','latex')
ylabel('Attitude Error [deg]','Interpreter','latex')
title('Quaternion Rotation Angle Error','Interpreter','latex')

%% =========================================================
% Nonlinear Attitude Dynamics
%% =========================================================
function dx = attitude_dynamics(x, u, I)

    q = x(1:4);
    w = x(5:7);

    Omega = [ 0    -w(1) -w(2) -w(3);
              w(1)  0     w(3) -w(2);
              w(2) -w(3)  0     w(1);
              w(3)  w(2) -w(1)  0 ];

    q_dot = 0.5 * Omega * q;
    w_dot = I \ (u - cross(w, I*w));

    dx = [q_dot; w_dot];
end
