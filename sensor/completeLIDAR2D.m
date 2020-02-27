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


% Completes the missing 2D LIDAR range data, due to limited angular sensing
% range, assuming the region outside the angular range is unoccupied.
%
% Function:
%   [R, LIDAR] = completeLIDAR2D(R, LIDAR)
%
% Input:
%   R           : LIDAR Range Data 
%   LIDAR       : LIDAR Parameters
%                   LIDAR.Range        - Range
%                   LIDAR.Infinity     - No detection 
%                   LIDAR.MinAngle     - Minimum angle
%                   LIDAR.MaxAngle     - Maximum angle
%                   LIDAR.NumSample    - Number of samples
%                   LIDAR.Resolution   - Angular resolution
%                   LIDAR.Angle        - Sampling angles%
% Output:
%   R          : Completed LIDAR Range Data
%   LIDAR      : Update LIDAR model
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
%   X = [5.5, 4.5, 0];
%   R = readLIDAR2D(X, LIDAR, Map);
%   [Rnew, LIDARnew] = completeLIDAR2D(R, LIDAR);
%   figure, hold on;
%   patch('XData', Map.Boundary(:,1), 'YData', Map.Boundary(:,2), 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2); 
%   for k = 1:numel(Map.Obstacle)
%       patch('XData', Map.Obstacle{k}(:,1), 'YData', Map.Obstacle{k}(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
%   end
%   [hFOV2, hRay2, hScatter2] = plotLIDAR2D(X, Rnew, LIDARnew, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   set(hFOV2, 'FaceColor', 'g', 'EdgeColor', 'g');
%   set(hRay2, 'Color', 'g');
%   set(hScatter2, 'CData', [0 1 0]);
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   axis equal, grid on, box on;
%   Rect = [min(Map.Boundary,[],1); max(Map.Boundary,[],1)];
%   set(gca, 'XLim', Rect(:,1), 'YLim', Rect(:,2)); 

function [R, LIDAR] = completeLIDAR2D(R, LIDAR)
% Author(s): 
%   Omur Arslan         omur@seas.upenn.edu
% Date: January 18, 2017

% Complete missing range measurements assuming the region outside the
% LIDAR's angular sensing range is free
if ((LIDAR.MaxAngle - LIDAR.MinAngle) < 2*pi)
    tempR = R; 
    tempAngle = LIDAR.Angle;
    tempResolution = LIDAR.Resolution;
    
    % Updated LIDAR model
    LIDAR.MaxAngle = LIDAR.MinAngle + 2*pi;
    LIDAR.NumSample = ceil((2*pi/tempResolution) + 1);
    LIDAR.Resolution = (LIDAR.MaxAngle - LIDAR.MinAngle)/(LIDAR.NumSample - 1);
    LIDAR.Angle = linspace(LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample);
    % Completed Range Data
    R = LIDAR.Infinity*ones(1,LIDAR.NumSample);
    I = floor(mod(tempAngle - LIDAR.MinAngle + LIDAR.Resolution/2,2*pi)/ LIDAR.Resolution) + 1;
    R(I) = tempR;
    R(LIDAR.NumSample) = R(1);
end
    