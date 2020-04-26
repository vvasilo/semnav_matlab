% MIT License (modified)
%
% Copyright (c) 2020 The Trustees of the University of Pennsylvania
% Authors:
% Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this **file** (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.


function [ode, out] = getHandles_unicycle_vf()
    ode = @unicycle_vf;
    out = @output_function;
    
    global pathTransformed
    global pathGoalTransformed
    
    pathTransformed = [];
    pathGoalTransformed = [];
    GoalTransformed = [];
    
    function dy = unicycle_vf(t,y)

        % Global Simulation Variables
        global Map; % Workspace Model
        global Robot; % Robot Model
        global Goal; % Goal Location

        % Set goal transformed equal to actual goal
        GoalTransformed = Goal;

        % Robot Configuration
        RobotState = y(:)';
        RobotPosition = [y(1), y(2)];
        RobotOrientation = y(3);

        % Read LIDAR data and compute projected goals for linear and angular motion
        R = readLIDAR2D(RobotState, Robot.Camera, Map);
        [PGL, PGA1, PGA2] = projgoalLIDAR2Dunicycle(RobotState, R, Robot.Camera, Robot.Radius, Goal);
        PGA = (PGA1 + PGA2)/2;

        % Compute the control inputs
        tV = (PGL - RobotPosition) * [cos(RobotOrientation); sin(RobotOrientation)];
        tW1 = (PGA - RobotPosition) * [cos(RobotOrientation); sin(RobotOrientation)];
        tW2 = (PGA - RobotPosition) * [-sin(RobotOrientation); cos(RobotOrientation)];
        dV = Robot.LinearCtrlGain * tV;
        dW = Robot.AngularCtrlGain * atan2(tW2, tW1);
        t
        dy = [dV*cos(RobotOrientation), dV*sin(RobotOrientation), dW]';
    end

    function status = output_function(t,y,flag)
        switch flag
            case 'init'
                pathTransformed = [pathTransformed ; [y(1) y(2) y(3)]];
                pathGoalTransformed = [pathGoalTransformed ; GoalTransformed];
            case ''
                pathTransformed = [pathTransformed ; [y(1) y(2) y(3)]];
                pathGoalTransformed = [pathGoalTransformed ; GoalTransformed];
            case 'done'
%                 assignin('base','pathTransformed',pathTransformed);
        end
        status = 0;
    end
end
