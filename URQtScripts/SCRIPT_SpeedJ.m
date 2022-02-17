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

%% Wait for the last SpeedJ to end
while ur.isMoving
    % Wait...
end

%% Move to home configuration
ur.Home;

%% Create a more interesting path (joint space)
% Set initial joint configuration
q0 = ur.Joints;
% Set final joint configuration
qf = zeros(6,1);

% Define movement time
t0 = 0;
delta_q = norm(qf - q0,inf);
tf = delta_q/dq_max;

% Fit continuous trajectory
t = [t0,tf];
q = [q0,qf];
dq = zeros(6,2);
ddq = zeros(6,2);
pp = fitpp(t,q,t,dq,t,ddq);

% Plot trajectory
tt = linspace(t0,tf,1000);
plotpp(pp,tt,2);

%% Execute move
ur.JointAcc = 2;
ur.Joints = q0;

dpp = diffpp(pp);
ddpp = diffpp(dpp);

dt = 0.05;
t = t0:dt:tf;
dq = ppval(dpp,t);
ddq = ppval(ddpp,t);

ur.BlockingTime = 2*dt;

for i = 1:size(dq,2)
    ur.JointAcc = norm(ddq(:,i),inf);
    dq_now = dq(:,i);
    % Execute ServoJ
    ur.SpeedJ(dq_now);
    % Wait for dt
    pause(0.8*dt);
end

% -------------------------------------------------------------------------
% END COMPLETED WORK
% -------------------------------------------------------------------------

%% Create a more interesting path (task space)
% Define end-effector poses for each joint configuration
H0 = UR_fkin(ur.URmodel,q0);
Hf = UR_fkin(ur.URmodel,qf);
% Define task configurations for each pose
v0 = pose2task(H0);
vf = pose2task(Hf);

% Fit continuous trajectory
t = [t0,tf];
v = [v0,vf];
dv = zeros(6,2);
ddv = zeros(6,2);
pp = fitpp(t,v,t,dv,t,ddv);

% Plot trajectory
tt = linspace(t0,tf,1000);
plotpp(pp,tt,2);

%% Map trajectory to joint space
dt = 0.05;
t = t0:dt:tf;
v = ppval(pp,t);

q = q0;
for i = 1:size(v,2)
    H = task2pose(v(:,i));
    q_all = UR_ikin(ur.URmodel,H);
    
    % Find closest configuration to q0
    d = zeros(1,size(q_all,2));
    for j = 1:size(q_all,2)
        d(j) = norm(q_all - q(:,end),inf);
    end
    idx = find(d == min(d));
    q(:,end+1) = q_all(:,idx(1));
end

figure;
axes; hold on
for i = 1:size(q,1)
    plot(t,q(i,:));
end

%% Define internal function(s)
function v = pose2task(H)
% POSE2TASK maps a pose to task space using a decoupled log map of SO(3)

R = H(1:3,1:3);
d = H(1:3,4);
k = vee( logSO(R) );

v = [k; d];
end

function H = task2pose(v)
% TASK2POSE maps a task space configuration to end-effector pose using a
% decoupled exponential map of SO(3)

k = v(1:3,:);
d = v(4:6,:);
R = expSO( wedge(k) );

H = eye(4);
H(1:3,1:3) = R;
H(1:3,4) = d;
end