classdef ButterflyCatcher
    % ButterflyCatcher
    % This class defines an elbow manipulator with a spherical wrist. This robot
    % is used to catch a butterfly.
    properties
        num_joints
        mass
        inertia
        com
        a
        alpha
        d
        catch_threshold
    end
    
    properties (Access = private)
        Tb_oArray
        joint_angle
        joint_torque
        DH_table;
        model_mass
        model_inertia
        model_com
        control_gains
        butterfly_caught
    end
    
    methods
%% initialization

        % ButterflyCatcher
        % param:
        %   num_jointsP - integer number of joints in robot arm
        %   massP - num_jointsx1 vector; massP(i) = mass of link i
        %   inertiaP - num_jointsx1 vector; inertiaP(i) = moment of inertia
        %       of link i
        %   comP - num_jointsx1 vector; comP(i) = distance of center of mass
        %       of link i from joint i-1
        %   aP - num_jointsx1 vector; aP(i) = value of a in DH table for
        %       link i
        %   alphaP = num_jointsx1 vector; alphaP(i) = value of alpha in DH
        %       table for link i
        %   dP = num_jointsx1 vector; dP(i) = value of d in DH table for
        %       link i
        % return:
        %   this = newly constructed ButterflyCatcher
        function this = ButterflyCatcher(num_jointsP, massP, inertiaP, ...
                comP, aP, alphaP, dP, catch_thresholdP)
            if nargin == 8
                this.num_joints = num_jointsP;
                this.mass = massP;
                this.inertia = inertiaP;
                this.com = comP;
                this.a = aP;
                this.alpha = alphaP;
                this.d = dP;
                this.catch_threshold = catch_thresholdP;
            else
                % default parameters
                this.num_joints = 6;
                this.mass = [1,1,1,1,1,1];
                this.inertia = [1,1,1,1,1,1];
                this.com = [0,0.5,0.5,0,0,0];
                this.a = [0,1,0,0,0,0.25];
                this.alpha = [pi/2,0,-pi/2,pi/2,-pi/2,0];
                this.d = [1,0,0,1.25,0,0];
                this.catch_threshold = 0.13;
            end
            this.DH_table = zeros(this.num_joints, 4);
            for i = 1:this.num_joints
                this.DH_table(i,1) = this.a(i);
                this.DH_table(i,2) = this.alpha(i);
                this.DH_table(i,3) = this.d(i);
            end
            this.butterfly_caught = 0;
        end
        
        % Setters
        function this = setTb_oArray(this, Tb_oArrayP)
            this.Tb_oArray = Tb_oArrayP;
        end
        function this = setJointAngles(this, joint_angleP)
            this.joint_angle = joint_angleP;
            this = reconstructDHTable(this);
        end
        function this = setJointTorque(this, joint_torqueP)
            this.joint_torque = joint_torqueP;
        end
        function this = setModelMass(this, model_massP)
            this.model_mass = model_massP;
        end
        function this = setModelInertia(this, model_inertiaP)
            this.model_inertia = model_inertiaP;
        end
        function this = setModelCom(this, model_comP)
            this.model_com = model_comP;
        end
        function this = setControlGains(this, control_gainsP)
            this.control_gains = control_gainsP;
        end
        
        % Getters
        function value = getJointAngles(this)
            value = this.joint_angle;
        end
        function value = getJointTorque(this)
            value = this.joint_angle;
        end
        function value = getModelMass(this)
            value = this.model_mass;
        end
        function value = getModelInertia(this)
            value = this.model_inertia;
        end
        function value = getModelCom(this)
            value = this.model_com;
        end
        function value = getControlGains(this)
            value = this.control_gains;
        end
        
%% forward kinematics
        
        % constructDHTable
        function this = reconstructDHTable(this)
            this.DH_table(:,4) = this.joint_angle(:);
        end
        
        % calcAi(DH_table,i) Given a DH table and index i, create matrix Ai
        % param:
        %     i: link index
        % return:
        %     Ai: 4x4 transformation matrix
        function Ai = calcAi(this,i)
            ai = this.DH_table(i,1);
            alphai = this.DH_table(i,2);
            di = this.DH_table(i,3);
            thetai = this.DH_table(i,4);
            Ai = [cos(thetai) -sin(thetai)*cos(alphai) sin(thetai)*sin(alphai) ai*cos(thetai); ...
                  sin(thetai) cos(thetai)*cos(alphai) -cos(thetai)*sin(alphai) ai*sin(thetai); ...
                  0 sin(alphai) cos(alphai) di; ...
                  0 0 0 1];
        end
        
        % calc_joint_positions
        % computes the positions of all joints in the robot
        % return:
        %   joint_positions - joint_positions(i) is a vector [x_i, y_i,
        %       z_i] giving the position of joint i in 3d space
        function joint_positions = calc_joint_positions(this)
            transforms = cell(this.num_joints,1);
            joint_positions = zeros(this.num_joints,3);

            transforms{1} = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]*this.calcAi(1);
            d_vec = transforms{1}*[0;0;0;1];
            joint_positions(1,:) = transpose(d_vec(1:3));

            for i = 2:this.num_joints
                transforms{i} = transforms{i-1}*this.calcAi(i);
                d_vec = transforms{i}*[0;0;0;1];
                joint_positions(i,:) = transpose(d_vec(1:3));
            end 
        end
%% inverse kinematics

        % calc_qd_pc
        % calculates the desired joint angles for the robot to be prepared
        % to catch the butterfly
        % param
        %   Tb_o - transform between the butterfly and the origin at time
        %   of catch
        %   qd_ic - vector of initial joint angles
        % return
        %   qd_pc - vector of prepared-to-catch joint angles
        function qd_pc = calc_qd_pc(this, Tb_o, qd_ic)
            qd_pc = zeros(6,1);
            
            % Position of the wrist center relative to the world origin
            Pwc_o = Tb_o*[0;0;-this.DH_table(6,1);1];
            
            % Calculate the two possible inverse position kinematics
            % solutions of the wrist, and choose the configuration closest
            % to the initial joint positions
            x_w = Pwc_o(1); y_w = Pwc_o(2); z_w = Pwc_o(3);
            
            % Joint 1
            qd_pc(1)=atan2(real(y_w),real(x_w));
            
            d1 = this.DH_table(1,3);
            a2 = this.DH_table(2,1);
            d4 = this.DH_table(4,3);
            
            % Joint 3
            D=real((x_w^2+y_w^2+(z_w-d1)^2-a2^2-d4^2)/(2*a2*d4));
            Theta3_1=atan2(real(sqrt(1-D^2)),D);
            Theta3_2=atan2(real(-sqrt(1-D^2)),D);

            % Joint 2
            Theta2_1=atan2(z_w-d1,sqrt(x_w^2+y_w^2))-atan2(d4*sin(Theta3_1),a2+d4*cos(Theta3_1));
            Theta2_2=atan2(z_w-d1,sqrt(x_w^2+y_w^2))-atan2(d4*sin(Theta3_2),a2+d4*cos(Theta3_2));

            if(abs(qd_ic(2,1)-Theta2_1)>abs(qd_ic(2,1)-Theta2_2))
                qd_pc(2)=Theta2_2;
                qd_pc(3)=Theta3_2-pi/2;
            else
                qd_pc(2)=Theta2_1;
                qd_pc(3)=Theta3_1-pi/2;
            end

            % Theta4,5
            %S1=(this.calcAi(3)'*this.calcAi(2)'*this.calcAi(1)')*T_B*[0;-1;0;0];
            S1=(this.calcAi(3)*this.calcAi(2)*this.calcAi(1))*Tb_o*[0;-1;0;0];

            S1_1=S1(1,1); S1_2=S1(2,1); S1_3=S1(3,1);

            if(or(S1_3==1,S1_3==-1))
                qd_pc(5)=atan2(0,S1_3);
                qd_pc(4)=qd_ic(4,1);
            else
                qd_pc(4)=atan2(-1*S1_2,-1*S1_1);
                qd_pc(5)=acos(S1_3);
            end
            
            % Theta6
            %S2=(this.calcAi(5)'*this.calcAi(4)'*this.calcAi(3)'*this.calcAi(2)'*this.calcAi(1)')*T_B*[1;0;0;0];
            S2=(this.calcAi(5)*this.calcAi(4)*this.calcAi(3)*this.calcAi(2)*this.calcAi(1))*Tb_o*[1;0;0;0];
            S2_1=S2(1,1);
            S2_2=S2(2,1);
            if(S2_2>=0)
                qd_pc(6)=acos(S2_1); 
            else
                qd_pc(6)=-acos(S2_1);
            end
        end
        
        % return_catch_angles
        % returns [qd_catch, qd_dot_catch, qd_ddot_catch] for each joint for when the robot
        % is swinging its net to catch the butterfly
        function qd_catch = return_catch_angles(~, qd_pc, tp_step, tf_step)
            time_range = tf_step-tp_step;
            qd_catch = zeros(6,time_range);
            
            angle_displacement = pi;
            
            angle_fraction = angle_displacement/time_range;
            qd_catch(:,1) = qd_pc;
            for t = 2:time_range
                qd_catch(:,t) = qd_catch(:,t-1) + [0;0;0;0;0;angle_fraction];
            end
        end


%% motion planning
        
        % motionPlanning
        % this plans a motion using a 3rd order polynomial based on
        % initial and final conditions. It returns the motion
        % expressions symbolically.
        % params:
        %   q_ic_d - q_ic_d(i) is the desired initial angle for joint i
        %   q_dot_ic_d - q_dot_ic_d(i) is the desired initial angular
        %       velocity for joint i
        %   q_fc_d - q_fc_d(i) is the desired final angle for joint i
        %   q_dot_fc_d - q_dot_fc_d(i) is the desired final angular velocity for
        %       joint i
        %   ts - start time
        %   tf - end time
        % return:
        %   qd_eq - qd_eq(i) is a symbolic expression for angle of joint i over time
        %   qd_dot_eq - qd_dot_eq(i) is a symbolic expression for angular
        %       velocity of joint i over time
        %   qd_ddot_eq - qd_ddot_eq(i) is a symbolic expression for angular
        %       acceleration of joint i over time
        function [qd_eq, qd_dot_eq, qd_ddot_eq] = motionPlanning(this, qd_ic, qd_fc, qd_dot_ic, qd_dot_fc,ts,tf)
            qd_eq = sym(zeros(this.num_joints, 1));
            qd_dot_eq = sym(zeros(this.num_joints, 1));
            qd_ddot_eq = sym(zeros(this.num_joints, 1));
            
            A = [ts^3    ts^2  ts  1;
                 tf^3    tf^2  tf  1;
                 3*ts^2  2*ts  1   0;
                 3*tf^2  2*tf  1   0];
            
            syms T
            for i = 1 : this.num_joints
                b_i = [qd_ic(i);qd_fc(i);qd_dot_ic(i);qd_dot_fc(i)];
                coeffs = A\b_i;
                qd_eq(i,:) = coeffs(1)*T^3 + coeffs(2)*T^2 + coeffs(3)*T + coeffs(4);
                qd_dot_eq(i,:) = 3*coeffs(1)*T^2 + 2*coeffs(2)*T + coeffs(3);
                qd_ddot_eq(i,:) = 6*coeffs(1)*T + 2*coeffs(2);
            end
        end
        
        % motionEvaluation
        % params:
        %   qd_eq - qd_eq(i) is a symbolic expression for angle of joint i over time
        %   qd_dot_eq - qd_dot_eq(i) is a symbolic expression for angular
        %       velocity of joint i over time
        %   qd_ddot_eq - qd_ddot_eq(i) is a symbolic expression for angular
        %       acceleration of joint i over time
        %   t_vec - t_vec is a vector containing the times at which we want
        %       to evaluate the symbolic expressions
        % return:
        %   qd - qd(i) is a horizontal vector of joint i angle at each time step in t_vec
        %   qd_dot - qd_dot(i) is a horizontal vector of joint i angular velocity at each time step i in t_vec
        %   qd_ddot - qd_ddot(i) is a horizontal vector of joint i angular acceleration at each time step i in t_vec
        function [qd, qd_dot, qd_ddot] = motionEvaluation(this, qd_eq, qd_dot_eq, qd_ddot_eq, t_vec)
            syms T
            qd = zeros(this.num_joints,length(t_vec));
            qd_dot = zeros(this.num_joints,length(t_vec));
            qd_ddot = zeros(this.num_joints,length(t_vec));
            for i = 1:this.num_joints
                for t = 1:length(t_vec)
                    qd(i,t) = double(subs(qd_eq(i),T,t_vec(t)));
                    qd_dot(i,t) = double(subs(qd_dot_eq(i),T,t_vec(t)));
                    qd_ddot(i,t) = double(subs(qd_ddot_eq(i),T,t_vec(t)));
                end
            end
        end
        
%% plotting
        function plotLink(~,o1,o2,color,size)
            plot3(o1(1),o1(2),o1(3),'ko',...
                'markersize',size,'markerfacecolor',color)
            plot3(o2(1),o2(2),o2(3),'ko',...
                'markersize',size,'markerfacecolor',color)
            link = [o1; o2];
            line(link(:,1),link(:,2),link(:,3),'linewidth',3)
        end
        
        function this = drawButterfly(this,t,joint_positions)
            hold on
            grid on
            b_xyz = [this.Tb_oArray{t}(1,4) this.Tb_oArray{t}(2,4) this.Tb_oArray{t}(3,4)];
            disp(norm(b_xyz-joint_positions(6,:)));
            if norm(b_xyz-joint_positions(6,:)) < this.catch_threshold || this.butterfly_caught == 1
                b_xyz = joint_positions(6,:);
                this.butterfly_caught = 1;
            end
            plot3(b_xyz(1),b_xyz(2),b_xyz(3),':r*','markersize',15,'markerfacecolor','r');
            orient_vec = this.Tb_oArray{t}*[0.15;0;0;1];
            this.plotLink(b_xyz,orient_vec(1:3).','y',7);
            hold off
        end
        
        % drawRobot
        function this = drawRobot(this, joint_positions)
            hold on
            grid on
            for i = 1:this.num_joints-1;
                this.plotLink(joint_positions(i,:), joint_positions(i+1,:),'k',15);
            end
            plot3(joint_positions(6,1),joint_positions(6,2),joint_positions(6,3), ...
                'o','markersize',18,'markerfacecolor','m');
            hold off
        end

        % animateMotion
        % param:
        %   dt - time to pause for in motion animation
        function this = animateMotion(this,t,t_done,dt,xspeed)
            % set axis limits [xmin xmax ymin ymax zmin zmax]
            axis([0 3 0 3 0 3],'manual');
            joint_positions = this.calc_joint_positions();
            this = this.drawRobot(joint_positions);
            this = this.drawButterfly(t,joint_positions);
            view(55,20);
            drawnow
            pause(dt/xspeed)     % pause with a 'correct' timing
            if t ~= t_done
                clf                 % clear current figure window
            end
        end
        
    end
end