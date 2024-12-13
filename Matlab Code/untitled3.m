
  % Constants
sigma_0 = 1e5; % Nm/rad
sigma_1 = sqrt(1e5); % Nms/rad
q_dot_s = 0.001; % rad/s, sliding speed threshold
sigma_3 = 0.4; % viscous friction coefficient
f_c = 1; % N, Coulomb friction (assumed for UR5)
f_s = 1.5; % Stribeck parameter (assumed for UR5)
jv=-0.0318;
% Size of the third dimension of jv
n = size(jv, 3);

% Initialize z and zdot for each joint and each time step
z = (f_c / sigma_0) ; % Initial bristle deflection [6x1xn]
 % Initial bristle velocity [6x1xn]

% Preallocate output torque vector
T_f = zeros(6, 1, n); % Friction torque [6x1xn]

% Calculate friction torque for each value of omega and each time step
%for k = 1:n
%    for i = 1:6
        % Bristle deflection dynamics (z_dot equation)
        pow = (jv/ q_dot_s) ^ 2;
        g_q_dot = (1 / sigma_0) * (f_c + (f_s - f_c) * exp(-pow));
        zdot = jv - (abs(jv) / g_q_dot) * z;

        % Friction torque Tf
        T_f = sigma_0 * z + sigma_1 * zdot + sigma_3 * jv;

        % Update bristle deflection for the next time step
        

