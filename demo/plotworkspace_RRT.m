% MIT License (modified)
%
% Copyright (c) 2020 The Trustees of the University of Pennsylvania
% Authors:
% Vasileios Vasilopoulos <vvasilo@seas.upenn.edu>
% Omur Arslan <omur@seas.upenn.edu>
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


function ax = plotworkspace_RRT(ScenarioID)

Scenario = scenario(ScenarioID);
Option = option(ScenarioID);

Boundary = Scenario.Map.Boundary;
Obstacle = Scenario.Map.Obstacle;
KnownObstacle = Scenario.Map.KnownObstacle;
Robot = Scenario.Robot;

% Bounding box
xMin = min(Boundary(:,1)) - Robot.Radius;
xMax = max(Boundary(:,1)) + Robot.Radius;
yMin = min(Boundary(:,2)) - Robot.Radius;
yMax = max(Boundary(:,2)) + Robot.Radius;

set(gcf, Option.FigureRRT{:});
ax = gca;
set(ax, Option.AxisRRT{:});
hold on;

axes(ax);
% Boundary Line
patch('XData', Boundary(:,1), 'YData', Boundary(:,2), Option.Boundary{:});

% Workspace Boundary
[Ex, Ey] = polybool('minus', [xMin xMin xMax xMax]', [yMin yMax yMax yMin]', Boundary(:,1), Boundary(:,2));
[f, v] = poly2fv(Ex, Ey);
patch('Faces', f, 'Vertices', v, Option.BoundaryObstacle{:});

% Obstacles
for k = 1: numel(Obstacle)
    if Obstacle{k}.Type==0
        patch('XData', Obstacle{k}.Points(:,1), 'YData', Obstacle{k}.Points(:,2), Option.UnknownObstacle{:});
    else
        patch('XData', Obstacle{k}.Points(:,1), 'YData', Obstacle{k}.Points(:,2), Option.KnownObstacle{:});
    end
end

































