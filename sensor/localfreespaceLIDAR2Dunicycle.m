% MIT License (modified)
%
% Copyright (c) 2020 The Trustees of the University of Pennsylvania
% Authors:
% Omur Arslan <omur@seas.upenn.edu>
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


% Computes the local freespace of a disk-shaped unicycle robot using 2D LIDAR range measurements
%
% Function:
%   [LFL, LFA1, LFA2] = localfreespaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal)
%
% Input:
%   X([x,y,th]) : LIDAR position (x,y) and orientation (th)
%   R           : Range Data at the LIDAR position
%   LIDAR       : LIDAR Parameters
%                   LIDAR.Range        - Range
%                   LIDAR.Infinity     - No detection 
%                   LIDAR.MinAngle     - Minimum angle
%                   LIDAR.MaxAngle     - Maximum angle
%                   LIDAR.NumSample    - Number of samples
%                   LIDAR.Resolution   - Angular resolution
%                   LIDAR.Angle        - Sampling angles
%   RobotRadius : Robot Radius
%   Goal        : Goal Position
%
% Output:
%   LFL          : Robot's Local Freespace for Linear Motion
%   LFA1, LFA2   : Robot's Local Freespace for Angular Motion
%
% Usage:
%   LIDAR.Range = 3;
%   LIDAR.Infinity = 1e5;
%   LIDAR.MinAngle = -0.75*pi;
%   LIDAR.MaxAngle = 0.75*pi;
%   LIDAR.NumSample = 100;
%   LIDAR.Resolution = (LIDAR.MaxAngle - LIDAR.MinAngle)/(LIDAR.NumSample - 1);
%   LIDAR.Angle = linspace(LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample);
%   Map.Boundary  = [0 0; 0 10; 10 10; 10 0; 0 0];
%   Map.Obstacle{1} = [2 2; 2 4; 4 4; 4 2; 2 2];
%   Map.Obstacle{2} = [6 6; 6 8; 8 8; 8 6; 6 6];
%   RobotRadius = 0.5;
%   X = [5.5, 4.5, 0];
%   Goal = [9, 9];
%   R = readLIDAR2D(X, LIDAR, Map);
%   [Rnew, LIDARnew] = completeLIDAR2D(R, LIDAR);
%   LW = localworkspaceLIDAR2D(X, R, LIDAR, RobotRadius);
%   [LFL, LFA1, LFA2] = localfreespaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal);
%   figure, hold on;
%   patch('XData', Map.Boundary(:,1), 'YData', Map.Boundary(:,2), 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2); 
%   for k = 1:numel(Map.Obstacle)
%       patch('XData', Map.Obstacle{k}(:,1), 'YData', Map.Obstacle{k}(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
%   end
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, Rnew, LIDARnew, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   set(hFOV, 'FaceColor', 'g', 'EdgeColor', 'g');
%   plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   I = localminLIDAR2D(R, LIDAR);
%   scatter(X(1)+R(I).*cos(LIDAR.Angle(I)+X(3)), X(2)+R(I).*sin(LIDAR.Angle(I)+X(3)), [], 'b', 'filled'); 
%   axis equal, grid on, box on;
%   Rect = [min(Map.Boundary,[],1); max(Map.Boundary,[],1)];
%   set(gca, 'XLim', Rect(:,1), 'YLim', Rect(:,2));
%   patch('XData', LW(:,1), 'YData', LW(:,2), 'FaceColor', 'y', 'EdgeColor', 'y', 'FaceAlpha', 0.5); 
%   patch('XData', LFA1(:,1), 'YData', LFA1(:,2), 'FaceColor', 'g', 'EdgeColor', 'g', 'FaceAlpha', 0.5); 
%   patch('XData', Goal(1) + RobotRadius*cos(linspace(-pi,pi,20)), 'YData', Goal(2) + RobotRadius*sin(linspace(-pi,pi,20)), 'FaceColor', 'm', 'EdgeColor', 'none'); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(-pi,pi,20)), 'YData', X(2) + RobotRadius*sin(linspace(-pi,pi,20)), 'FaceColor', 'b', 'EdgeColor', 'none'); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) + RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 
%   patch('XData', X(1) - RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) - RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 
%   plot(LFA2(:,1), LFA2(:,2), 'm', 'LineWidth', 2); 
%   plot(LFL(:,1), LFL(:,2), 'c', 'LineWidth', 2); 

function [LFL, LFA1, LFA2] = localfreespaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal)
% Author: Omur Arslan, omur@seas.upenn.edu
% Date : January 19, 2017

LFA1 = localfreespaceLIDAR2D(X, R, LIDAR, RobotRadius);
if isempty(LFA1)
    LFA2 = [];
    LFL = [];
else
    RobotPosition = [X(1), X(2)];
    RobotOrientation = X(3);
    RobotDirection = [cos(RobotOrientation), sin(RobotOrientation)];
    [LFLx, LFLy] = polyxray(LFA1(:,1), LFA1(:,2), RobotPosition, RobotDirection);
    LFL = [RobotPosition; LFLx(:), LFLy(:)];
    GoalDirection = [(Goal(1) - RobotPosition(1)), (Goal(2) - RobotPosition(2))];
    [LFA2x, LFA2y] = polyxray(LFA1(:,1), LFA1(:,2), RobotPosition, GoalDirection);
    LFA2 = [RobotPosition; LFA2x(:), LFA2y(:)];
end
