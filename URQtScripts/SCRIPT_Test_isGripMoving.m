%% SCRIPT_Test_isGripMoving
% This script assumes the UR is already initialized and in remote mode. The
% URQt object should be "ur".
%
%   M. Kutzer, 20May2021, USNA

ur.GripPosition = 0; % Open gripper
pause(5);

ur.GripSpeed = 255;
ur.GripPosition = 52;
while true
    g0 = ur.GripPosition;
    pause(0.1);
    g1 = ur.GripPosition;
    if g0 == g1
        break
    else
        fprintf('Gripper Moving\n');
    end
end
fprintf('Gripper Finished\n');
