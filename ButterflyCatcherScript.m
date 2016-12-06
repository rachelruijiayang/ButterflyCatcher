clear
clc
close all

%% define robot and butterfly parameters and initial conditions
robot = ButterflyCatcher();  % Default parameters
animateSwitch = 'on';

t_start = 1;
t_prepared = 2.5;
t_finish = 3;
fps = 15;
t_vec = linspace(t_start,t_finish,(t_finish-t_start)*fps);
t_sp = t_vec(1:(t_prepared-t_start)*fps);
t_pf = t_vec((t_prepared-t_finish)*fps:t_finish*fps);

Tb_oArray = cell(length(t_vec), 1);   % cell array containing transform from butterfly to world origin at each timestep
Tb_oArray{1} = zeros(4,4);
Tb_oArray{1}(1:3,1:3) = [1,0,0;0,1,0;0,0,1];
Tb_oArray{1}(1:3,4) = [1.5;1.5;1.5];
Tb_oArray{1}(4,:) = [0,0,0,1];

% butterfly movement
for t = 2:length(t_sp)
    Tb_oArray{t} = zeros(4,4);
    Tb_oArray{t}(1:3,1:3) = [1,0,0;0,1,0;0,0,1];
    Tb_oArray{t}(1:3,4) = Tb_oArray{t-1}(1:3,4) + -0.05 + 0.1*rand(3,1);
    Tb_oArray{t}(4,:) = [0,0,0,1];
end
robot = robot.setTb_oArray(Tb_oArray);

qd_ic = [0;0;0;0;0;0];
qd_fc = robot.calc_qd_fc(Tb_oArray{(t_finish-t_start)*fps},qd_ic);
qd_dot_ic = [0;0;0;0;0;0];
qd_dot_fc = [0;0;0;0;0;0];

%% motion planning
[qd_eq, qd_dot_eq, qd_ddot_eq] = robot.motionPlanning(qd_ic, qd_fc, qd_dot_ic, qd_dot_fc, t_start, t_finish);
[qd, qd_dot, qd_ddot] = robot.motionEvaluation(qd_eq, qd_dot_eq, qd_ddot_eq, t_vec);

figure
for t = 2:length(t_vec)
    robot = robot.setJointAngles(qd(:,t));
    robot.animateMotion(t,t_vec(t)-t_vec(t-1));
end
close gcf