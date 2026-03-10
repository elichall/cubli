%% --- Main ---
clear;close all;clc; format long

Constants;

%% --- Find Max Safe Rotational Speed and Torque ---
w_theoreticalMax = rating * batteryVoltage; 

w_max = 0.9 * w_theoreticalMax; % buffer for safety

w_maxRad = w_max * RPM_TO_RADPS;

Kt = 60 / (2*pi * rating); % motor constant

I_stall = (batteryVoltage - voltageSafetyFactor) / R_ph; 
if (I_stall<I_driverMax)
    I_max = I_stall;
else 
    I_max = I_driverMax;
end

T_max = Kt * I_max;

%% --- Find Required Wheel Inertial Mass ---

E_wheel = I_wheel * w_maxRad^2 / 2; % energy of wheel

n_efficiency = I_wheel / (Iyy_edgeWheelMax + Iyy_edgeMax); % interial transfer efficiency ratio

E_barrier = (3 * m_wheel + m_body) * g * S * (sqrt(2) - 1) / 2; % energy required to overcome the PE diff

LHS = E_wheel * n_efficiency * lossFactor; % loss factor accounts for energy lost to the motor through heat and sound
RHS = E_barrier;

inequ = LHS >= RHS;
%disp(n_efficiency)
%disp([E_wheel, E_barrier])
%disp([LHS RHS])
%disp(inequ);

%% --- Compute K Matrix ---

% need to tune 
Qe = [1/(angleMax)^2 0 0 0;
          0 1/(angVelMax)^2 0 0;
          0 0 1/(motorAngleMax)^2 0;
          0 0 0 1/(w_maxRad)^2];

Re = [1/T_max^2];

[Ke_max, Pe_max, Ee_max] = lqr(Ae_max, Be_max, Qe, Re);
disp(Ke_max)

[Ke_min, Pe_min, Ee_min] = lqr(Ae_min, Be_min, Qe, Re);
disp(Ke_min)
% K is the gain matrix, S is the solution to the algebraic Riccati equation, and P are the closed-loop poles.


