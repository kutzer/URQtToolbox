%% SCRIPT_SpeedJ
% This script tests the *.SpeedJ command using a specified blocking time
% and leading axis acceleration.
%
% This SCRIPT assumes the user has already run:
%   ur = URQt;
%   ur.Initialize
%
%   M. Kutzer. 17Feb2022, USNA

%% Move to home configuration
ur.Home;

%% Update blocking time & gain
ur.BlockingTime = 0.5;  % s
ur.JointAcc = 2;        % rad/s^2

%% Test ServoJ command
% Define peak speed
dq_max = deg2rad(5);
for i = 1:10
    % Define random 6-DoF unit vector
    v = rand(6,1);
    v_hat = v./norm(v);
    
    % Define random speed command
    dq = dq_max*v_hat;
    
    % Execute ServoJ
    ur.SpeedJ(dq);
    % Wait for a portion of the blocking time
    pause(0.7*ur.BlockingTime);
end