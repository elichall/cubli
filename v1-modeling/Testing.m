%% angle transform test
close all;clear;clc;

rotmToEuler = @(R) [ atan2( R(3, 2), R(3, 3) );
                     atan2( -R(3, 1), sqrt( power(R(1, 1), 2) + power(R(2, 1), 2) ));
                     atan2( R(2, 1), R(1, 1) ) ];

sensorToNbody = [ 0, -1,  0;
                 -1,  0,  0;
                  0,  0, -1 ];

vbodyToEdge = [sqrt(2)/2, 0, sqrt(2)/2; 
               0, 1, 0;
              -sqrt(2)/2, 0, sqrt(2)/2];



disp(rad2deg(rotmToEuler(sensorToNbody))')
disp(rad2deg(rotmToEuler(vbodyToEdge))')
disp(vbodyToEdge * sensorToNbody)

start = [-137.27 ; -3.05 ; -7.30];

final = rad2deg(rotmToEuler(vbodyToEdge * sensorToNbody * eul2rotm(deg2rad(start'), "XYZ"),"XYZ"))

% --- State and Instance Found by getOrientation ---
% 1 | 0
% --- RAW Angles ---
% -137.27 | -3.05 | -7.30
% --- State Vector Angles ---
% -2.27 | 7.32 | -86.99




