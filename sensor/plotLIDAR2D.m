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


% Draws the range map of a 2D LIDAR sensor
%
% Function:
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'off', 'Scatter', 'off')
%
% Inputs:  
%   X([x,y,th]) : Position (x,y) and Orientation (theta) of the LIDAR
%   R           : Range reading of a LIDAR at the specified position
%   LIDAR       : LIDAR Parameters
%
% Optional Inputs:
%   'FOV'       : Flag variable for the Filed-of-View plot, which can be turned 'on' or 'off' 
%   'Ray'       : Flag variable for the Line-of-Sight rays, which can be turned 'on' or 'off'    
%   'Scatter'   : Flag variable for the Scatter plot, which can be turned 'on' or 'off'
%
% Outputs: 
%   hFOV        : Handle for the Field-Of-Field (FOV) plot
%   hRay        : Handle for the LIDAR rays
%   hScatter    : Gandle for the LIDAR scatter plot
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
%   figure, hold on;
%   patch('XData', Map.Boundary(:,1), 'YData', Map.Boundary(:,2), 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2); 
%   for k = 1:numel(Map.Obstacle)
%       patch('XData', Map.Obstacle{k}(:,1), 'YData', Map.Obstacle{k}(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
%   end
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   axis equal, grid on, box on;
%   Rect = [min(Map.Boundary,[],1); max(Map.Boundary,[],1)];
%   set(gca, 'XLim', Rect(:,1), 'YLim', Rect(:,2)); 

function [hFOV, hRay, hScatter] = plotLIDAR2D(X, R, LIDAR, varargin)
% Author(s): 
%   Omur Arslan         omur@seas.upenn.edu
% Date: November 05, 2016

    % Visualization options
    flagFOV = 1; % Flag variable for LIDAR Field-of-View 
    flagRay = 0; % Flag variable for LIDAR Line-of-Sight Rays
    flagScatter = 0; % Flag variable for scatter plot
    
    % Check inputs for the visualization options
    for k = 1:(numel(varargin)-1)
        switch lower(varargin{k})
            case 'fov'
                switch lower(varargin{k+1})
                    case 'on'
                        flagFOV = true;
                    case 'off'
                        flagFOV = false;
                end
            case 'ray'
                switch lower(varargin{k+1})
                    case 'on'
                        flagRay = true;
                    case 'off'
                        flagRay = false;
                end
            case 'scatter'
                switch lower(varargin{k+1})
                    case 'on'
                        flagScatter = true;
                    case 'off'
                        flagScatter = false;
                end
        end              
    end
    
    % LIDAR Parameters
    Position = [X(1) X(2)]; % LIDAR Position
    Orientation = X(3); % LIDAR Orientation
    Angle = LIDAR.Angle + Orientation; % LIDAR Angle Samples
    ValidRange = (R <= LIDAR.Range); % Valid Range Data
    R(~ValidRange) = LIDAR.Range; % Range Data
    
    % Plot Field-of-View (FOV)
    hFOV = [];
    if flagFOV
        optionFOV = {'FaceColor', 'r', 'EdgeColor', 'r', 'FaceAlpha', 0.1, 'LineWidth', 1};
        if (LIDAR.MaxAngle - LIDAR.MinAngle)< (2*pi)
            hFOV = patch('XData', Position(1) + [0, R.*cos(Angle)], 'YData', Position(2) + [0, R.*sin(Angle)], optionFOV{:});
        else
            hFOV = patch('XData', Position(1) + R.*cos(Angle), 'YData', Position(2) + R.*sin(Angle), optionFOV{:});
        end
    end
    
    % Plot LIDAR Line-of-Sight Rays
    hRay = [];
    if flagRay
        optionRay = {'Color', 'r', 'LineWidth', 1};
        numRay = sum(ValidRange);
        hRay = plot(Position(1)+ [zeros(1, numRay); R(ValidRange).*cos(Angle(ValidRange))], Position(2)+ [zeros(1, numRay); R(ValidRange).*sin(Angle(ValidRange))], optionRay{:});
    end
    
    % LIDAR Scatter Plot
    hScatter = [];
    if flagScatter
       optionScatter = {'r', 'filled'};
       hScatter = scatter(Position(1) + R(ValidRange).*cos(Angle(ValidRange)), Position(2) + R(ValidRange).*sin(Angle(ValidRange)), [], optionScatter{:});
    end
end