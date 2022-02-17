%% SCRIPT_ServoJ
% This script tests the *.ServoJ command using a specified blocking time
% and gain.
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
ur.Gain = 100;          % proportional gain

%% Test ServoJ command
for i = 1:10
    % Get current joint configuration
    q0 = ur.Joints;
    % Create updated position
    qf = q0 + deg2rad(5)*ones( size(q0) );
    
    % Execute ServoJ
    ur.ServoJ(qf);
    % Wait for a portion of the blocking time
    pause(0.7*ur.BlockingTime);
end
