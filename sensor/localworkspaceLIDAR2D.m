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


% Computes the local workspace of a disk-shaped robot using 2D LIDAR range data
%
% Function:
%   LW = localworkspaceLIDAR2D(X, R, LIDAR, RobotRadius)
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
%
% Output:
%   LW          : Local Workspace polygon 
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
%   R = readLIDAR2D(X, LIDAR, Map);
%   [Rnew, LIDARnew] = completeLIDAR2D(R, LIDAR);
%   LW = localworkspaceLIDAR2D(X, R, LIDAR, RobotRadius);
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
%   patch('XData', LW(:,1), 'YData', LW(:,2), 'FaceColor', 'y', 'EdgeColor', 'y', 'FaceAlpha', 0.5); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(-pi,pi,20)), 'YData', X(2) + RobotRadius*sin(linspace(-pi,pi,20)), 'FaceColor', 'b', 'EdgeColor', 'none'); 
%   patch('XData', X(1) + RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) + RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 
%   patch('XData', X(1) - RobotRadius*cos(linspace(pi/6,5*pi/6,10)), 'YData', X(2) - RobotRadius*sin(linspace(pi/6,5*pi/6,10)), 'FaceColor', 'k', 'EdgeColor', 'none'); 

function LW = localworkspaceLIDAR2D(X, R, LIDAR, RobotRadius)
% Author(s): 
%   Omur Arslan         omur@seas.upenn.edu
% Date: January 18, 2017

epsilon = 1e-10;
if (min(R) < epsilon)
    LW = [];
    return;
end

% Complete missing data due to the LIDAR's angular range limits
[R, LIDAR] = completeLIDAR2D(R, LIDAR); 

% Limit the LIDAR sensing range
R(R > LIDAR.Range) = LIDAR.Range; 

% Modified range data defining the local workspace using the separating hyperplane
% between each data point and the robot body
R = 0.5*(R + RobotRadius);

% Compute local workspace 
LW = (0.5*(LIDAR.Range+RobotRadius))*[-1 -1; -1 1; 1 1; 1 -1]; % Initialize the local workspace with the minimum square that respect the LIDAR sensing range 
Imin = find(localminLIDAR2D(R, LIDAR)); %  The indices of local minima of range data
for k = Imin
    if isempty(LW)
        return;
    else
        %Local minimum parameters
        Ak = LIDAR.Angle(k); % Angle
        Rk = R(k); % Range

        % Separating hyperplane parameters
        n = -[cos(Ak+X(3)), sin(Ak+X(3))]; % Separating hyperplane normal
        m = - Rk*n; % A point on the separating hyperplane
        
        %Update the local workspace by taking its intersection with the associated halfplane 
        [xi, yi] = polyxhplane(LW(:,1),LW(:,2), m, n);
        LW = [xi, yi];
    end
end

%Local Workspace footprint 
LocalFootprint = flipud([R.*cos(LIDAR.Angle+X(3)); R.*sin(LIDAR.Angle+X(3))]');
%Update Local workspace
[xi, yi] = polybool('intersection',LW(:,1), LW(:,2), LocalFootprint(:,1), LocalFootprint(:,2));
LW = [xi+X(1), yi+X(2)];

% A numerical trick to make local workspace convex. Otherwise, it requires
% a dilation of the robot's local free space
k = convhull(LW(:,1), LW(:,2));
LW = [LW(k,1), LW(k,2)];