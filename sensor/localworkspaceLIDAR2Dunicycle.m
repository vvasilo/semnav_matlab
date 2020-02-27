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


% Computes the local workspace of a disk-shaped unicycle robot using 2D LIDAR range measurements 
%
% Function:
%   [LWL, LWA1, LWA2] = localworkspaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal)
%
% Input:
%   X([x,y,th]) : LIDAR Position (x,y) and Orientation (th)
%   R           : Range Data at the LIDAR Position
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
%   LWL          : Local Workspace Polygon for Linear Motion
%   LWA1, LWA2   : Local Workspace Polygon for Angular Motion
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
%   [LWL, LWA1, LWA2] = localworkspaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal);
%   figure, hold on;
%   patch('XData', Map.Boundary(:,1), 'YData', Map.Boundary(:,2), 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2); 
%   for k = 1:numel(Map.Obstacle)
%       patch('XData', Map.Obstacle{k}(:,1), 'YData', Map.Obstacle{k}(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
%   end
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, Rnew, LIDARnew, 'FOV', 'on', 'Ray', 'off', 'Scatter', 'off');
%   set(hFOV, 'FaceColor', 'g', 'EdgeColor', 'g');
%   plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   I = localminLIDAR2D(R, LIDAR);
%   scatter(X(1)+R(I).*cos(LIDAR.Angle(I)+X(3)), X(2)+R(I).*sin(LIDAR.Angle(I)+X(3)), [], 'b', 'filled'); 
%   axis equal, grid on, box on;
%   Rect = [min(Map.Boundary,[],1); max(Map.Boundary,[],1)];
%   set(gca, 'XLim', Rect(:,1), 'YLim', Rect(:,2));
%   patch('XData', LWA1(:,1), 'YData', LWA1(:,2), 'FaceColor', 'y', 'EdgeColor', 'y', 'FaceAlpha', 0.5); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(-pi,pi,20)), 'YData', X(2) + RobotRadius*sin(linspace(-pi,pi,20)), 'FaceColor', 'b', 'EdgeColor', 'none'); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) + RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 
%   patch('XData', X(1) - RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) - RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 
%   patch('XData', Goal(1) + RobotRadius*cos(linspace(-pi,pi,20)), 'YData', Goal(2) + RobotRadius*sin(linspace(-pi,pi,20)), 'FaceColor', 'm', 'EdgeColor', 'none'); 
%   plot(LWA2(:,1), LWA2(:,2), 'm', 'LineWidth', 2); 
%   plot(LWL(:,1), LWL(:,2), 'c', 'LineWidth', 2); 

function [LWL, LWA1, LWA2] = localworkspaceLIDAR2Dunicycle(X, R, LIDAR, RobotRadius, Goal)
% Author: Omur Arslan, omur@seas.upenn.edu
% Date : January 19, 2017

LWA1 = localworkspaceLIDAR2D(X, R, LIDAR, RobotRadius);

if isempty(LWA1)
    LWA2 = [];
    LWL = [];
else
    RobotPosition = [X(1), X(2)];
    RobotOrientation = X(3);
    RobotDirection = [cos(RobotOrientation), sin(RobotOrientation)];
    [LWLx, LWLy] = polyxray(LWA1(:,1), LWA1(:,2), RobotPosition, RobotDirection);
    LWL = [RobotPosition; LWLx(:), LWLy(:)];
    GoalDirection = [(Goal(1) - RobotPosition(1)), (Goal(2) - RobotPosition(2))];
    [LWA2x, LWA2y] = polyxray(LWA1(:,1), LWA1(:,2), RobotPosition, GoalDirection);
    LWA2 = [RobotPosition; LWA2x(:), LWA2y(:)];
end
