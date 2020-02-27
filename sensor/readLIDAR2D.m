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


% Simulates a 2D LIDAR range sensor in a planar environment with obstacles
% 
% Function:
%   R = readLIDAR2D(X, LIDAR, Map)
%
% Inputs:
%   X ([x y th]): Location (x,y) and Orientation (th) of a LIDAR sensor
%   LIDAR       : LIDAR Parameters
%                   LIDAR.Range        - Range
%                   LIDAR.Infinity     - No detection 
%                   LIDAR.MinAngle     - Minimum angle
%                   LIDAR.MaxAngle     - Maximum angle
%                   LIDAR.NumSample    - Number of samples
%                   LIDAR.Resolution   - Angular resolution
%                   LIDAR.Angle        - Sampling angles
%   Map        : A map of a polygonal environment with obstacles
%                   Map.Boundary   -  Boundary (nx2 matrix) 
%                   Map.Obstacle   -  List of Obstacles (a cell array)
% Output:
%   R          : Range Data
% Usage:
%   LIDAR.Range = 3;
%   LIDAR.Infinity = 1e5;
%   LIDAR.MinAngle = -0.75*pi;
%   LIDAR.MaxAngle = 0.75*pi;
%   LIDAR.NumSample = 100;
%   LIDAR.Resolution = (LIDAR.MaxAngle - LIDAR.MinAngle)/(LIDAR.NumSample - 1);
%   LIDAR.Angle = linspace(LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample);
%   Map.Boundary  = [0 0; 0 10; 10 10; 10 0; 0 0];
%   Map.Obstacle{1}.Points = [2 2; 2 4; 4 4; 4 2; 2 2];
%   Map.Obstacle{2}.Points = [6 6; 6 8; 8 8; 8 6; 6 6];
%   X = [5.5, 4.5, 0.5];
%   R = readLIDAR2D(X, LIDAR, Map);
%   figure, hold on;
%   patch('XData', Map.Boundary(:,1), 'YData', Map.Boundary(:,2), 'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 2); 
%   for k = 1:numel(Map.Obstacle)
%       patch('XData', Map.Obstacle{k}.Points(:,1), 'YData', Map.Obstacle{k}.Points(:,2), 'FaceColor', 'k', 'EdgeColor', 'none');
%   end
%   [hFOV, hRay, hScatter] = plotLIDAR2D(X, R, LIDAR, 'FOV', 'on', 'Ray', 'on', 'Scatter', 'on');
%   axis equal, grid on, box on;
%   Rect = [min(Map.Boundary,[],1); max(Map.Boundary,[],1)];
%   set(gca, 'XLim', Rect(:,1), 'YLim', Rect(:,2)); 

function R = readLIDAR2D(X, LIDAR, Map)
% Author(s): 
%   Omur Arslan         omur@seas.upenn.edu
% Date: November 04, 2016

    Position = [X(1) X(2)]; % Position
    Orientation = X(3); % Orientation
    
    R = zeros(1, LIDAR.NumSample); % Memory allocation and initialization
    % Check if the position is outside the workspace boundary
    if ~inpolygon(Position(1), Position(2), Map.Boundary(:,1), Map.Boundary(:,2))
        return;
    end
    % Check if the position is inside an obstacle
    for co = 1:numel(Map.Obstacle)
        if inpolygon(Position(1), Position(2), Map.Obstacle{co}.Points(:,1), Map.Obstacle{co}.Points(: ,2))
            return;
        end
    end
    
    % Rotation matrix from the global frame to the local sensor frame
    RotMat = [cos(-Orientation), -sin(-Orientation); sin(-Orientation), cos(-Orientation)]; 

    % Determine distance to the workspace boundary and obstacles
    R = LIDAR.Infinity*ones(1, LIDAR.NumSample);  % Infinity is modeled as a very large value & Memory allocation for the range data
    Map.Obstacle{end+1}.Points = Map.Boundary; % Include Workspace boundary as an obstacle
    for co = 1:numel(Map.Obstacle)
        % Obstacle in the local sensor frame
        Obs = bsxfun(@minus, Map.Obstacle{co}.Points, Position);
        Obs = Obs*RotMat';
        % Compute distance to every obstacle edge 
        for cv = 1:size(Obs,1)
            cn = mod(cv, size(Obs,1)) + 1; % Next vertex index
            vc = Obs(cv,:); % Current Vertex
            vn = Obs(cn,:); % Next Vertex
            
            %Compute the distance to the origin
            dist = min(norm(vc), norm(vn));
            w = (vn*(vn - vc)')/(norm(vn-vc)^2);
            if (w>=0)&&(w<=1)
                vx = w*vc + (1-w)*vn;
                dist = min(dist, norm(vx));
            end
            
            ac = atan2(vc(2), vc(1));  % Relative angle of the current vertex
            an = atan2(vn(2), vn(1));  % Relative angle of the next vertex
            
            flagDist = (dist <= LIDAR.Range);
            flagAngle = (min(ac,an)<= LIDAR.MaxAngle) && (max(ac,an) >= LIDAR.MinAngle);
            % Compute LIDAR range if the obstacle segment is in the sensing region 
            if (flagDist && flagAngle)
                % Closest LIDAR ray index
                I = round((max(min(ac,an), LIDAR.MinAngle) - LIDAR.MinAngle)/LIDAR.Resolution) + 1;
                I = mod(I-1, LIDAR.NumSample) + 1;
                % Compute the intersection of the LIDAR ray with the sensor footprint 
                vtemp = [cos(LIDAR.Angle(I)), sin(LIDAR.Angle(I))];
                vRtemp = [-sin(LIDAR.Angle(I)), cos(LIDAR.Angle(I))];
                w = -(vn*vRtemp')/((vc - vn)*vRtemp');
                if (w>=0)&&(w<=1)        
                    xtemp = w*vc + (1 - w)*vn;
                    if (xtemp*vtemp') >= 0
                        R(1,I) = min(R(1,I), norm(xtemp));
                    end
                end
                % Compute the intersection of adjacent LIDAR rays
                J = mod(I, LIDAR.NumSample) + 1;
                flagValid = 1;
                while flagValid && (J ~= I)
                    vtemp = [cos(LIDAR.Angle(J)), sin(LIDAR.Angle(J))];
                    vRtemp = [-sin(LIDAR.Angle(J)), cos(LIDAR.Angle(J))];
                    w = -(vn*vRtemp')/((vc - vn)*vRtemp');
                    if (w>=0)&&(w<=1)        
                        xtemp = w*vc + (1 - w)*vn;
                        if (xtemp*vtemp') >= 0
                            R(1,J) = min(R(1,J), norm(xtemp));
                            J = mod(J, LIDAR.NumSample) + 1;
                        else
                            flagValid = 0;
                        end
                    else
                        flagValid = 0;
                    end
                end

                J = mod(I-2, LIDAR.NumSample) + 1;
                flagValid = 1;
                while flagValid && (J ~= I)
                    vtemp = [cos(LIDAR.Angle(J)), sin(LIDAR.Angle(J))];
                    vRtemp = [-sin(LIDAR.Angle(J)), cos(LIDAR.Angle(J))];
                    w = -(vn*vRtemp')/((vc - vn)*vRtemp');
                    if (w>=0)&&(w<=1)        
                        xtemp = w*vc + (1 - w)*vn;
                        if (xtemp*vtemp') >= 0
                            R(1,J) = min(R(1,J), norm(xtemp));
                            J = mod(J-2, LIDAR.NumSample) + 1;
                        else
                            flagValid = 0;
                        end
                    else
                        flagValid = 0;
                    end
                end  
            end
        end
    end
    % Check sensor range
    R(R > LIDAR.Range) = LIDAR.Infinity;    
end

