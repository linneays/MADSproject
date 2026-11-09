classdef DiffDriveSimulator < handle
    % DiffDriveSimulator Simulates a differential drive robot
    %   This class simulates pose and path drawing in real time
    
    % NOTA: va ripristinato inizializzazione ma con path giusto inizio non 0
    
    properties
        robotRadius
        % enableLaser
        robotCurrentPose   % [x, y, theta]
        robotGoal          % The target location [x, y]
        actualPath         % The whole path until the current time step
        
        % These are defined while calling the simulation functions:
%         b % baseline
%         Rl % right wheel radius
%         Rr % left wheel radius
        
        Enc_res  % encoder Tic/revolution
        K_param  % [baseline, right wheel radius, left wheel radius]
        DK_param % deviations from ideal parameters
        
        fig          % Handle to figure
        ax           % Handle to axis
        
    end

    methods

        function obj = DiffDriveSimulator...
                (K_param, DK_param, Enc_res, Pose)
            % Constructor for UnicycleSimulator
            % Initialize the robot's properties

            obj.K_param = K_param;
            obj.DK_param = DK_param;
            obj.Enc_res = Enc_res;

            obj.robotCurrentPose = Pose;  % Set initial pose
            obj.actualPath = Pose(1:2);
        end

        function Show(obj, path, Nplot, Xlim, Ylim)
            %% Method for Diff Drive Simulator
            % Generate a figure, which is cleared at every call of the
            % method. 
            % Plot the input path.
            % Plot the real path computed by the unicycle.
            figure(Nplot), plot([Xlim(1) Xlim(2) Xlim(2) Xlim(1) Xlim(1)], ...
                [Ylim(1) Ylim(1) Ylim(2) Ylim(2) Ylim(1)],'k', 'LineWidth', 4)
            hold on
            % cla; % Clear
            plot(path(:,1), path(:,2),'b--d', 'LineWidth', 4, 'MarkerSize', 10, ...
                'MarkerEdgeColor','r', 'MarkerFaceColor',[0.5,0.5,0.5]);
            axis equal
            grid on
            plot(obj.actualPath(:,1), obj.actualPath(:,2));
            obj.drawUnicycle;
            drawnow;
            % hold off;
        end

        function Pose = GrabPose(obj)
            %% Method for Diff Drive Simulator
            % Grab the robot pose
            Pose = obj.robotCurrentPose;
        end

        
        function Path = GrabPath(obj)
            %% Method for Diff Drive Simulator
            % Grab the robot pose
            Path = obj.actualPath;
        end

        
        % kinematic simulation with l/r wheels Velocity
        function Simulate(obj, dt, Vr, Vl)
            
            b = obj.K_param(1);
            
            current_pose = obj.robotCurrentPose;
            % Diff Drive Model
            % [   xdot   ]   [ cos(delta)/2     cos(delta)/2 ]  [  Vr  ]
            % [   ydot   ] = [ sin(delta)/2     sin(delta)/2 ]  [  Vl  ]
            % [ deltadot ]   [     1/b          -1/b ]
            J = [cos(current_pose(3))/2 cos(current_pose(3))/2;...
                 sin(current_pose(3))/2 sin(current_pose(3))/2;...
                 1/b                    -1/b];
            derivative_pose = J * [Vr; Vl];
            xdot = derivative_pose(1);
            ydot = derivative_pose(2);
            deltadot = derivative_pose(3);

            x_new = current_pose(1) + xdot * dt;
            y_new = current_pose(2) + ydot * dt;
            delta_new = current_pose(3) + deltadot * dt;

            obj.robotCurrentPose = [x_new, y_new, delta_new];
            obj.actualPath = [obj.actualPath; ...
                              x_new, y_new];

        end

        % kinematic simulation with l/r encoders Tic for each cycle
        % Takes as parameters: K_param + DK_param (deviations)
        function SimulateEnc(obj, dt, NTic_r, NTic_l)
            
            b = obj.K_param(1) + obj.DK_param(1);
            current_pose = obj.robotCurrentPose;
            
            Vr = (obj.K_param(2) + obj.DK_param(2)) * 2*pi * NTic_r / (obj.Enc_res * dt);
            Vl = (obj.K_param(3) + obj.DK_param(3)) * 2*pi * NTic_l / (obj.Enc_res * dt);
            
            % Diff Drive Model
            % [   xdot   ]   [ cos(delta)/2     cos(delta)/2 ]  [  Vr  ]
            % [   ydot   ] = [ sin(delta)/2     sin(delta)/2 ]  [  Vl  ]
            % [ deltadot ]   [     1/b          -1/b ]
            J = [cos(current_pose(3))/2 cos(current_pose(3))/2;...
                 sin(current_pose(3))/2 sin(current_pose(3))/2;...
                 1/b                    -1/b];
            derivative_pose = J * [Vr; Vl];
            xdot = derivative_pose(1);
            ydot = derivative_pose(2);
            deltadot = derivative_pose(3);

            x_new = current_pose(1) + xdot * dt;
            y_new = current_pose(2) + ydot * dt;
            delta_new = current_pose(3) + deltadot * dt;

            obj.robotCurrentPose = [x_new, y_new, delta_new];
            obj.actualPath = [obj.actualPath; ...
                              x_new, y_new];

        end

        function drawUnicycle(obj)
            % Draw the unicycle robot based on its current pose
            
            X = obj.robotCurrentPose(1);
            Y = obj.robotCurrentPose(2);
            Delta = obj.robotCurrentPose(3);
            
            % Triangle-like unicycle
            s = obj.K_param(1);
            hold on;
            % Draw body
            line([X+s*cos(Delta+(3*pi/5)), X+s*cos(Delta-(3*pi/5))],...
                [Y+s*sin(Delta+(3*pi/5)), Y+s*sin(Delta-(3*pi/5))], 'Color','k');
            line([X+s*cos(Delta+(3*pi/5)), X+1.5*s*cos(Delta)],...
                [Y+s*sin(Delta+(3*pi/5)), Y+1.5*s*sin(Delta)], 'Color','k');
            line([X+1.5*s*cos(Delta), X+s*cos(Delta-(3*pi/5))],...
                [Y+1.5*s*sin(Delta), Y+s*sin(Delta-(3*pi/5))],'Color','k');
            
            % Draw wheels
            b = obj.K_param(1);
            line([X + 0.707*b*cos(Delta+(pi/4)), X + 0.707*b*cos(Delta+(3*pi/4))],...
                 [Y + 0.707*b*sin(Delta+(pi/4)), Y + 0.707*b*sin(Delta+(3*pi/4))],...
                'LineWidth', 10, 'Color', 'k');
            line([X + 0.707*b*cos(Delta-(pi/4)), X + 0.707*b*cos(Delta-(3*pi/4))],...
                 [Y + 0.707*b*sin(Delta-(pi/4)), Y + 0.707*b*sin(Delta-(3*pi/4))],...
                'LineWidth', 10, 'Color', 'k');
            
            % Centre of the axis
            plot(X, Y, 'kx', 'MarkerSize', 20, 'LineWidth', 3, 'MarkerFaceColor', 'k');
            axis equal;
            
            hold off;
        end

        function setRobotPose(obj, CurrentPose)
            % Set the robot's current pose
            if numel(CurrentPose) == 3
                obj.robotCurrentPose = CurrentPose;
            else
                error('Pose must be a 1x3 vector [x, y, theta]');
            end
        end

    end
      
end
