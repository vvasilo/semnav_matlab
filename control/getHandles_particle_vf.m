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

function [ode, out] = getHandles_particle_vf()
    ode = @particle_vf;
    out = @output_function;
    
    global pathTransformed
    
    pathTransformed = [];
    
    function dy = particle_vf(t,y)

        % Global Simulation Variables
        global Map; % Workspace Model
        global Robot; % Robot Model
        global Goal; % Goal Location

        % Robot Configuration
        RobotState = y(:)';
        RobotPosition = [y(1), y(2)];
        RobotOrientation = y(3);

        % Read LIDAR data and compute projected goals for linear and angular motion
        R = readLIDAR2D(RobotState, Robot.Camera, Map);
        PG = projgoalLIDAR2D(RobotState, R, Robot.Camera, Robot.Radius, Goal);

        % Compute the control inputs
        tV = Robot.LinearCtrlGain*(PG - RobotPosition);
        dy = [tV(1), tV(2), 0]';
    end

    function status = output_function(t,y,flag)
        switch flag
            case 'init'
                pathTransformed = [pathTransformed ; [y(1) y(2) y(3)]];
            case ''
                pathTransformed = [pathTransformed ; [y(1) y(2) y(3)]];
            case 'done'
%                 assignin('base','pathTransformed',pathTransformed);
        end
        status = 0;
    end
end