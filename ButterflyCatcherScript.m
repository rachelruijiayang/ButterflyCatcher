clear
clc
close all

%% setup
robot = ButterflyCatcher();  % Default parameters

%% animation settings
fps = 20;
xspeed = .5;

%% time
% time benchmarks
t_start = 1;
t_prepared = 3.85;
t_finish = 4;

tp_step = round((t_prepared-t_start)*fps);
tf_step = round((t_finish-t_start)*fps);

% timestep vectors
t_vec = linspace(t_start,t_finish,tf_step);
t_sp = t_vec(1:tp_step);
t_pf = t_vec(tp_step:tf_step);

%% butterfly setup
Tb_oArray = cell(length(t_vec), 1);   % cell array containing transform from butterfly to world origin at each timestep
Tb_oArray{1} = zeros(4,4);
Tb_oArray{1}(1:3,1:3) = [1,0,0;0,1,0;0,0,1];
Tb_oArray{1}(1:3,4) = [1;1;1.3];
Tb_oArray{1}(4,:) = [0,0,0,1];

% butterfly movement
for t = 2:length(t_vec)
    Tb_oArray{t} = zeros(4,4);
    Tb_oArray{t}(1:3,1:3) = [1,0,0;0,1,0;0,0,1];
    Tb_oArray{t}(1:3,4) = Tb_oArray{t-1}(1:3,4) + -0.02+0.04*rand(3,1)+[0.004;0.004;0];
    Tb_oArray{t}(4,:) = [0,0,0,1];
end
robot = robot.setTb_oArray(Tb_oArray);

% set robot's initial, prepared-to-catch, and final conditions
qd_ic = [0;0;0;0;0;0];
qd_dot_ic = [0;0;0;0;0;0];
qd_pc = robot.calc_qd_pc(Tb_oArray{round((tp_step + tf_step)/2)},qd_ic);
qd_dot_pc = [0;0;0;0;0;0];

%% motion planning
qd = zeros(6,length(t_vec));
qd_dot = zeros(6,length(t_vec));
qd_ddot = zeros(6,length(t_vec));

% start to prepared-to-catch
[qd_sp_eq, qd_dot_sp_eq, qd_ddot_sp_eq] = robot.motionPlanning(qd_ic, qd_pc, qd_dot_ic, qd_dot_pc, t_start, t_prepared);
[qd(:,1:tp_step), qd_dot(:,1:tp_step), qd_ddot(:,1:tp_step)] = robot.motionEvaluation(qd_sp_eq, qd_dot_sp_eq, qd_ddot_sp_eq, t_vec(1:tp_step));
 
% prepared-to-catch to finish
qd(:,tp_step+1:tf_step) = robot.return_catch_angles(qd_pc,tp_step,tf_step);

%% robot animation
figure
for t = 2:length(t_vec)
    robot = robot.setJointAngles(qd(:,t));
    robot = robot.animateMotion(t,tf_step,t_vec(t)-t_vec(t-1),xspeed);
end

figure
plot(t_vec, qd(1,:), t_vec, qd(2,:),t_vec, qd(3,:), t_vec, qd(4,:),t_vec, qd(5,:), t_vec, qd(6,:));
title('Joint Angles over Time');
legend('Joint1','Joint2','Joint3','Joint4','Joint5','Joint6', 'Location','northwest');
xlabel('Time (s)');
ylabel('Joint Angle (rad)');
