%% SCRIPT_Test_FlushBuffer
% Script to cause NaN return from ur.Joints.
%
%   MIDN R. Cushing, 24Feb2022, USNA

ur = URQt('UR3e');
pause(2);
ur.Initialize;

%% Just hit Ctrl + C while the robot is moving to cause NaN return
while true
    temp=ur.Joints;
    ur.Zero;
    temp=ur.Joints;
    ur.Home;
end
% Use ur.Joints in command window to verify NaN's are being received,
% then enter ur.FlushBuffer to clear, then enter ur.Joints again to verify
% clear.