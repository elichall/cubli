%% Define Physical Constants

% --- Physics Constants ---
g = 9.80665; % m/s^2
lossFactor = 0.7; % lost energy in jump
RPM_TO_RADPS = 0.10472;

% --- Component Constants ---
% Motor
R_ph = 16.4; % ohm, phase resistance 
rating = 90; % KV (rpm/V), rating

% Driver
I_driverMax = 2.5; % amps 

% Battery
batteryVoltage = 11.1; % V
voltageSafetyFactor = 1.1; 

% --- Geometry ---
S = 0.160; % m, side length
Le_ideal = S*sqrt(2)/2;
Le_min = 0.106;
Le_max = 0.120;

% --- Masses (kg) ---
m_body = 542.53 / 1000; 

m_wheel = 65.75 / 1000;
m_tot = m_body + 3 * m_wheel;

% --- Inertial Masses (kg*m^2) ---
% Body Frame
I_body = (0)/3 * 10^(-9); % w/out wheels average of Ixx, Iyy, Izz
I_wheel = 130940.82 * 10^(-9); % average inertial masses of isolated wheels from body

% Edge Frame
Iyy_edgeMin = 9795653.46 * 10^(-9); % w/out wheels, wheels closest to the edge
Iyy_edgeWheelMin = (972609.32 + 2 * 499771.42) * 10^(-9); % inertial masses of isolated wheels from edge (all three)

Iyy_edgeMid = (9129684.58 + 9445670.77+ 9545910.54) / 3 * 10^(-9); % w/out wheels, might need to average out multiple edges later or take extreme
Iyy_edgeWheelMid = (499771.42 + 1891185.35 + 972609.14) * 10^(-9); 

Iyy_edgeMax = 9313629.20 * 10^(-9); % w/out wheels, from the edge furthest from the motors
Iyy_edgeWheelMax = (972599.15 + 2 * 1891166.14) * 10^(-9);

% Corner Frame
I_corner = [0 0 0] * 10^(-9); % [Ixx Iyy Izz] w/ y-axis aligned with face
I_cornerWheel = [] * 10^(-9); % inertial masses of isolated wheels from corner (all three)

% --- LQR Constants ---
% Plant
Ae_max = [0 1 0 0;
          Le_max * g * m_tot / (Iyy_edgeMax+Iyy_edgeWheelMax) 0 0 0;
          0 0 0 1;
          0 0 0 0];
Be_max = [0;
         -1/(Iyy_edgeMax+Iyy_edgeWheelMax);
          0;
          1/I_wheel];

Ae_min = [0 1 0 0;
          Le_min * g * m_tot / (Iyy_edgeMin+Iyy_edgeWheelMin) 0 0 0;
          0 0 0 1;
          0 0 0 0];
Be_min = [0;
         -1/(Iyy_edgeMin+Iyy_edgeWheelMin);
          0;
          1/I_wheel];

% LQR Weights
angleMax = deg2rad(3); % 5 degree to stay within lineariaztion limits of sin(th) = th
angVelMax = deg2rad(30); % 30 deg/s penalizing harsher makes less occilations 
motorAngleMax = 50; % about 8 revolutions

