% Antenna Params
m = 0.3;               % Mass, [kg]
w = 0.1;             % Width, [m]
l = 0.1;              % Length, [m]
d = 0.1;              % Depth, [m]
A = w*l;              % Area, [m^2]
beta = pi/4;          % Elevation angle, [rad]
Kd = 5;               % Damping constant, [N*m/(rad/s)]
J = m/12*(l^2*cos(beta)^2+d^2*sin(beta)^2+w^2);  % Inertia, [kg*m^2]

% Motor parameters
Kf = 0.1;             % Back EMF constant, [V/(rad/s)]
Kt = 0.1;             % Torque constant, [N*m/A]
L = 1e-5;              % Inductance, [H]
R = 10;                % Resistance, [Ohm]
Kg = 2000;             % Gear ratio, []

% constants
r = 6371000;           % Earth's radius, [m]