%% TEST_GripPosition
% Test the wait associated with changes in grip position
%
%   M. Kutzer, 27Mar2025, USNA
clear all
close all
clc

%% Initialize robot
ur = URQt('UR3e',true);
ur.Initialize;

%% Change grip position
while true
    ur.GripPosition = 52;
    ur.GripPosition = 0;
end