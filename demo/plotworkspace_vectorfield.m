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


function [ax1,ax2,ax3,ax4] = plotworkspace_vectorfield(ScenarioID)

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

set(gcf, Option.Figure{:});
ax1 = subplot(141);
set(ax1, Option.Axis1{:});
hold on;
ax2 = subplot(142);
set(ax2, Option.Axis2{:});
hold on;
ax3 = subplot(143);
set(ax3, Option.Axis3{:});
hold on;
ax4 = subplot(144);
set(ax4, Option.Axis4{:});
hold on;

axes(ax1);
title('Physical Space', 'FontSize', 30, 'FontWeight', 'bold', 'Interpreter', 'latex');
% Boundary Line
patch('XData', Boundary(:,1), 'YData', Boundary(:,2), Option.Boundary{:});

% Augmented Workspace Boundary
[Fx, Fy] = cvxpolyerode(Boundary(:,1), Boundary(:,2), Robot.Radius);
[Bx, By] = polybool('minus', Boundary(:,1), Boundary(:,2), Fx, Fy);
[Bf, Bv] = poly2fv(Bx, By);

% Workspace Boundary
[Ex, Ey] = polybool('minus', [xMin xMin xMax xMax]', [yMin yMax yMax yMin]', Boundary(:,1), Boundary(:,2));
[f, v] = poly2fv(Ex, Ey);
patch('Faces', f, 'Vertices', v, Option.BoundaryObstacle{:});
patch('Faces', Bf, 'Vertices', Bv, Option.DiscoveredObstacleDilated{:});

% Obstacles
for k = 1: numel(Obstacle)
    if Obstacle{k}.Type==0
        patch('XData', Obstacle{k}.Points(:,1), 'YData', Obstacle{k}.Points(:,2), Option.UnknownObstacle{:});
    else
        patch('XData', Obstacle{k}.Points(:,1), 'YData', Obstacle{k}.Points(:,2), Option.KnownObstacle{:});
    end
end

axes(ax2);
title('Semantic Space', 'FontSize', 30, 'FontWeight', 'bold', 'Interpreter', 'latex');
% Boundary Line
patch('XData', Boundary(:,1), 'YData', Boundary(:,2), Option.Boundary{:});
% Augmented Workspace Boundary
[Fx, Fy] = cvxpolyerode(Boundary(:,1), Boundary(:,2), Robot.Radius);
[Bx, By] = polybool('minus', Boundary(:,1), Boundary(:,2), Fx, Fy);
[Bf, Bv] = poly2fv(Bx, By);

% Workspace Boundary
[Ex, Ey] = polybool('minus', [xMin xMin xMax xMax]', [yMin yMax yMax yMin]', Boundary(:,1), Boundary(:,2));
[f, v] = poly2fv(Ex, Ey);
patch('Faces', f, 'Vertices', v, Option.BoundaryObstacle{:});
patch('Faces', Bf, 'Vertices', Bv, Option.BoundaryObstacle{:});

axes(ax3);
title('Mapped Space', 'FontSize', 30, 'FontWeight', 'bold', 'Interpreter', 'latex');
% Boundary Line
patch('XData', Boundary(:,1), 'YData', Boundary(:,2), Option.Boundary{:});
% Augmented Workspace Boundary
[Fx, Fy] = cvxpolyerode(Boundary(:,1), Boundary(:,2), Robot.Radius);
[Bx, By] = polybool('minus', Boundary(:,1), Boundary(:,2), Fx, Fy);
[Bf, Bv] = poly2fv(Bx, By);

% Workspace Boundary
[Ex, Ey] = polybool('minus', [xMin xMin xMax xMax]', [yMin yMax yMax yMin]', Boundary(:,1), Boundary(:,2));
[f, v] = poly2fv(Ex, Ey);
patch('Faces', f, 'Vertices', v, Option.BoundaryObstacle{:});
patch('Faces', Bf, 'Vertices', Bv, Option.BoundaryObstacle{:});

axes(ax4);
title('Model Space', 'FontSize', 30, 'FontWeight', 'bold', 'Interpreter', 'latex');
% Boundary Line
patch('XData', Boundary(:,1), 'YData', Boundary(:,2), Option.Boundary{:});
% Augmented Workspace Boundary
[Fx, Fy] = cvxpolyerode(Boundary(:,1), Boundary(:,2), Robot.Radius);
[Bx, By] = polybool('minus', Boundary(:,1), Boundary(:,2), Fx, Fy);
[Bf, Bv] = poly2fv(Bx, By);

% Workspace Boundary
[Ex, Ey] = polybool('minus', [xMin xMin xMax xMax]', [yMin yMax yMax yMin]', Boundary(:,1), Boundary(:,2));
[f, v] = poly2fv(Ex, Ey);
patch('Faces', f, 'Vertices', v, Option.BoundaryObstacle{:});
patch('Faces', Bf, 'Vertices', Bv, Option.BoundaryObstacle{:});
end









































