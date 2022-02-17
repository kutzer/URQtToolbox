%% SCRIPT_ServoJ

ur.Home;
ur.BlockingTime = 0.5;

for i = 1:10
    q0 = ur.Joints;
    qf = q0 + deg2rad(5)*ones( size(q0) );
    
    ur.ServoJ(qf);
    pause(0.7*ur.BlockingTime);
end