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


% vectorField.m
% Demonstration of the vector field for a fixed set of known obstacles
function [] = vectorField(ScenarioID)
    %% Start with a clean simulation environment
    close all; % Close all figures
    clearvars -except ScenarioID; % Clear all variables
    rng(0); % Seeds the random number generator
    warning('off','all');
    %% Simulation Settings

    % Simulation Scenario:
    numSampleObstacle = 40; % Number of samples for representing obstacle surface
    numSampleLIDAR = 360; % Number of angular samples for LIDAR
    numVectorField = 30; % Number of points to plot for the vector field

    % Global Variables
    global Map;         % A convex polygonal environment populated with obstacles
    global KnownMap;    % Known semantic map to be used in the simulation
    global Robot;       % Robot Model
    global Goal;        % Goal Location
    Scenario = scenario(ScenarioID, numSampleObstacle, numSampleLIDAR); % Simulation Settings
    Option = option(ScenarioID); % Visualization Option

    %% Animate the resulting trajectory
    % Multimedia output settings
    filename = sprintf('multimedia/vector_field_%d', ScenarioID);
    flagSaveFigure = 1; % Flag variable for saving figure

    global pathTransformed; % Initializer for path in model space
    global pathGoalTransformed % Initializer for goal position array in model space

    figure
    [ax1,ax2,ax3,ax4] = plotworkspace_vectorfield(ScenarioID); % Plot workspace

    clear Map Robot Goal KnownMap
    global Map Robot Goal KnownMap
    % Reset the Map
    Map = Scenario.Map;
    Robot = Scenario.Robot;
    Goal = Scenario.Goal;

    % Initialize semantic map to be used in the simulation
    [Fx,Fy] = cvxpolyerode(Scenario.Map.Boundary(:,1), Scenario.Map.Boundary(:,2), Robot.Radius);
    KnownMap.Boundary = [Fx,Fy];
    KnownMap.SeenObstacles = [];
    KnownMap.FamiliarObstacles = [];

    % Initialize a map copy to be used locally for the animation
    MapCopy.Obstacle = Map.Obstacle;
    MapCopy.Boundary = Map.Boundary;
    MapCopy.KnownObstacle = Map.KnownObstacle;
    MapCopy.DiffeoParams = Map.DiffeoParams;
    DiffeoParameters = py.dict();
    DiffeoParameters{'p'} = MapCopy.DiffeoParams.p;
    DiffeoParameters{'epsilon'} = MapCopy.DiffeoParams.epsilon;
    DiffeoParameters{'varepsilon'} = MapCopy.DiffeoParams.varepsilon;
    DiffeoParameters{'mu_1'} = MapCopy.DiffeoParams.mu1;
    DiffeoParameters{'mu_2'} = MapCopy.DiffeoParams.mu2;
    DiffeoParameters{'workspace'} = py.numpy.array(py.numpy.round([MapCopy.Boundary ; MapCopy.Boundary(1,:)] + [Robot.Radius Robot.Radius ; Robot.Radius -Robot.Radius ; -Robot.Radius -Robot.Radius ; -Robot.Radius Robot.Radius ; Robot.Radius Robot.Radius],pyargs('decimals',int32(4))));

    % Initialize the semantic map to be used for the animation locally
    SemanticMap.Boundary = KnownMap.Boundary;
    SemanticMap.SeenObstacles = [];



    % Plot the physical workspace
    axes(ax1)
    hold on;

    % Plot goal
    hGoal = patch('XData', Goal(1)+Robot.Polygon(:,1), 'YData', Goal(2)+Robot.Polygon(:,2), Option.Goal{:});

    % Initialize list of unknown obstacles
    UnknownObstacles{:} = [];

    for i=1:numel(MapCopy.Obstacle)
        % Check whether it is known or unknown
        if (MapCopy.Obstacle{i}.Type ~= 0)
            % Add the obstacle to the list of seen obstacles
            SemanticMap.SeenObstacles = [SemanticMap.SeenObstacles, i];

            % Set as discovered patch
            hDiscoveredDilated(i) = patch('XData', MapCopy.Obstacle{i}.DilatedPoints(:,1), 'YData', MapCopy.Obstacle{i}.DilatedPoints(:,2), Option.DiscoveredObstacleDilated{:});
            hDiscovered(i) = patch('XData', MapCopy.Obstacle{i}.Points(:,1), 'YData', MapCopy.Obstacle{i}.Points(:,2), Option.DiscoveredObstacle{:});
        elseif (MapCopy.Obstacle{i}.Type == 0)
            % Dilate the obstacles and add only the visible portion in the map
            polyObstacle = polyshape(MapCopy.Obstacle{i}.Points(:,1),MapCopy.Obstacle{i}.Points(:,2));
            polyBuffer = polybuffer(polyObstacle,Robot.Radius,'JointType','round');
            x2 = flipud(polyBuffer.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyBuffer.Vertices(:,2)); y2 = [y2;y2(1)];
            UnknownObstacles{end} = [x2,y2];
            hDiscoveredDilated(i) = patch('XData', x2, 'YData', y2, Option.DiscoveredObstacleDilated{:});
            hDiscovered(i) = patch('XData', MapCopy.Obstacle{i}.Points(:,1), 'YData', MapCopy.Obstacle{i}.Points(:,2), Option.DiscoveredObstacle{:});
            UnknownObstacles{end+1} = [];
        end
    end
    UnknownObstacles(end) = [];

    % Find the union of all known obstacles
    polyUnion = polyshape();
    for j=1:length(SemanticMap.SeenObstacles)
        polyUnion = union(polyUnion, polyshape(MapCopy.Obstacle{SemanticMap.SeenObstacles(j)}.DilatedPoints));
    end

    % Split the result up to different polygons to find the familiar obstacles
    idx = all(isnan(polyUnion.Vertices),2);
    idy = 1+cumsum(idx);
    idz = 1:size(polyUnion.Vertices,1);
    SemanticMap.FamiliarObstacles = accumarray(idy(~idx),idz(~idx),[],@(r){polyUnion.Vertices(r,:)});

    % Calculate resulting trees
    for j=1:length(SemanticMap.FamiliarObstacles)
        SemanticMap.FamiliarObstacles{j} = [SemanticMap.FamiliarObstacles{j} ; SemanticMap.FamiliarObstacles{j}(1,:)];
        [xout, yout] = poly2ccw(SemanticMap.FamiliarObstacles{j}(:,1), SemanticMap.FamiliarObstacles{j}(:,2));
        SemanticMap.FamiliarObstacles{j} = [xout,yout];
        SemanticMap.FamiliarObstaclesTrees{j} = py.reactive_planner_lib.diffeoTreeConvex(py.numpy.array(py.numpy.round(SemanticMap.FamiliarObstacles{j},pyargs('decimals',int32(4)))), DiffeoParameters);
    end

    % Construct the model space based on the temporary map created above: the
    % model space will contain the equivalent spheres for the known obstacles
    % and the dilated visible portions of the unknown obstacles
    TemporaryMapModel.Boundary = SemanticMap.Boundary;
    TemporaryMapModel.Obstacle{:} = [];
    for i=1:numel(SemanticMap.FamiliarObstacles)
        TemporaryMapModel.Obstacle{end}.Points = obstacle2D_ball(double(SemanticMap.FamiliarObstaclesTrees{i}{end}{'center'}), double(SemanticMap.FamiliarObstaclesTrees{i}{end}{'radius'}), 100);
        TemporaryMapModel.Obstacle{end+1} = [];
    end
    for i=1:numel(UnknownObstacles)
        TemporaryMapModel.Obstacle{end}.Points = UnknownObstacles{i};
        TemporaryMapModel.Obstacle{end+1} = [];
    end
    TemporaryMapModel.Obstacle(end) = [];


    % Find the diffeomorphism and its jacobian at a given position, along
    % with the necessary second derivatives - Also find the goal
    % position in the model space
    xmin = SemanticMap.Boundary(1,1);
    ymin = SemanticMap.Boundary(1,2);
    xmax = SemanticMap.Boundary(3,1);
    ymax = SemanticMap.Boundary(3,2);
    for i=1:numVectorField+1
        for j=1:numVectorField+1
            % Find new point to examine
            point_to_examine = [xmin+(i-1)*(xmax-xmin)/numVectorField, ymin+(j-1)*(ymax-ymin)/numVectorField];

            % Check if the point belongs to any familiar or unknown obstacle and continue if that's the case
            flag_known = 0; flag_unknown = 0;
            for k=1:length(SemanticMap.FamiliarObstacles)
                if inpolygon(point_to_examine(1),point_to_examine(2),SemanticMap.FamiliarObstacles{k}(:,1),SemanticMap.FamiliarObstacles{k}(:,2))
                    flag_known = 1;
                    break;
                end
            end
            for k=1:length(UnknownObstacles)
                if inpolygon(point_to_examine(1),point_to_examine(2),UnknownObstacles{k}(:,1),UnknownObstacles{k}(:,2))
                    flag_unknown = 1;
                    break;
                end
            end
            if (flag_known) || (flag_unknown)
                continue;
            end

            % Find the position and jacobian in the model space
            RobotPositionTransformed = point_to_examine;
            RobotPositionTransformedD = eye(2);
            RobotPositionTransformedDD = zeros(1,8);
            GoalTransformed = Goal;
            for k=1:numel(SemanticMap.FamiliarObstacles)
                diffeo_result = py.reactive_planner_lib.polygonDiffeoConvex(py.numpy.array(RobotPositionTransformed), SemanticMap.FamiliarObstaclesTrees{k}, DiffeoParameters);
                TempPositionTransformed = double(diffeo_result{1});
                TempPositionTransformedD = double(diffeo_result{2});
                TempPositionTransformedDD = double(diffeo_result{3});

                res1 = TempPositionTransformedD(1,1)*RobotPositionTransformedDD(1) + TempPositionTransformedD(1,2)*RobotPositionTransformedDD(5) + RobotPositionTransformedD(1,1)*(TempPositionTransformedDD(1)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(2)*RobotPositionTransformedD(2,1)) + RobotPositionTransformedD(2,1)*(TempPositionTransformedDD(3)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(4)*RobotPositionTransformedD(2,1));
                res2 = TempPositionTransformedD(1,1)*RobotPositionTransformedDD(2) + TempPositionTransformedD(1,2)*RobotPositionTransformedDD(6) + RobotPositionTransformedD(1,1)*(TempPositionTransformedDD(1)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(2)*RobotPositionTransformedD(2,2)) + RobotPositionTransformedD(2,1)*(TempPositionTransformedDD(3)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(4)*RobotPositionTransformedD(2,2));
                res3 = TempPositionTransformedD(1,1)*RobotPositionTransformedDD(3) + TempPositionTransformedD(1,2)*RobotPositionTransformedDD(7) + RobotPositionTransformedD(1,2)*(TempPositionTransformedDD(1)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(2)*RobotPositionTransformedD(2,1)) + RobotPositionTransformedD(2,2)*(TempPositionTransformedDD(3)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(4)*RobotPositionTransformedD(2,1));
                res4 = TempPositionTransformedD(1,1)*RobotPositionTransformedDD(4) + TempPositionTransformedD(1,2)*RobotPositionTransformedDD(8) + RobotPositionTransformedD(1,2)*(TempPositionTransformedDD(1)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(2)*RobotPositionTransformedD(2,2)) + RobotPositionTransformedD(2,2)*(TempPositionTransformedDD(3)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(4)*RobotPositionTransformedD(2,2));
                res5 = TempPositionTransformedD(2,1)*RobotPositionTransformedDD(1) + TempPositionTransformedD(2,2)*RobotPositionTransformedDD(5) + RobotPositionTransformedD(1,1)*(TempPositionTransformedDD(5)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(6)*RobotPositionTransformedD(2,1)) + RobotPositionTransformedD(2,1)*(TempPositionTransformedDD(7)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(8)*RobotPositionTransformedD(2,1));
                res6 = TempPositionTransformedD(2,1)*RobotPositionTransformedDD(2) + TempPositionTransformedD(2,2)*RobotPositionTransformedDD(6) + RobotPositionTransformedD(1,1)*(TempPositionTransformedDD(5)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(6)*RobotPositionTransformedD(2,2)) + RobotPositionTransformedD(2,1)*(TempPositionTransformedDD(7)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(8)*RobotPositionTransformedD(2,2)); 
                res7 = TempPositionTransformedD(2,1)*RobotPositionTransformedDD(3) + TempPositionTransformedD(2,2)*RobotPositionTransformedDD(7) + RobotPositionTransformedD(1,2)*(TempPositionTransformedDD(5)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(6)*RobotPositionTransformedD(2,1)) + RobotPositionTransformedD(2,2)*(TempPositionTransformedDD(7)*RobotPositionTransformedD(1,1) + TempPositionTransformedDD(8)*RobotPositionTransformedD(2,1));
                res8 = TempPositionTransformedD(2,1)*RobotPositionTransformedDD(4) + TempPositionTransformedD(2,2)*RobotPositionTransformedDD(8) + RobotPositionTransformedD(1,2)*(TempPositionTransformedDD(5)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(6)*RobotPositionTransformedD(2,2)) + RobotPositionTransformedD(2,2)*(TempPositionTransformedDD(7)*RobotPositionTransformedD(1,2) + TempPositionTransformedDD(8)*RobotPositionTransformedD(2,2));
                RobotPositionTransformedDD(1) = res1;
                RobotPositionTransformedDD(2) = res2;
                RobotPositionTransformedDD(3) = res3;
                RobotPositionTransformedDD(4) = res4;
                RobotPositionTransformedDD(5) = res5;
                RobotPositionTransformedDD(6) = res6;
                RobotPositionTransformedDD(7) = res7;
                RobotPositionTransformedDD(8) = res8;

                RobotPositionTransformedD = TempPositionTransformedD*RobotPositionTransformedD;

                RobotPositionTransformed = TempPositionTransformed;

                diffeo_result = py.reactive_planner_lib.polygonDiffeoConvex(py.numpy.array(GoalTransformed), SemanticMap.FamiliarObstaclesTrees{k}, DiffeoParameters);
                GoalTransformed = double(diffeo_result{1});
            end

            % Construct a virtual LIDAR with 2*pi span to be used in the model space
            % It is important to keep the LIDAR range smaller than the camera range to
            % ensure continuity
            LIDAR.Range = Robot.Camera.LidarRange;
            LIDAR.Infinity = 60;
            LIDAR.MinAngle = -pi;
            LIDAR.MaxAngle = pi;
            LIDAR.NumSample = Robot.Camera.NumSample;
            LIDAR.Resolution = (LIDAR.MaxAngle - LIDAR.MinAngle)/(LIDAR.NumSample - 1);
            LIDAR.Angle = linspace(LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample);

            % Read LIDAR data and compute projected goal in model space; the robot
            % radius can be zero because we have already dilated the obstacles
            R = readLIDAR2D([RobotPositionTransformed, 0], LIDAR, TemporaryMapModel);
            PG = projgoalLIDAR2D([RobotPositionTransformed, 0], R, LIDAR, 0, GoalTransformed);

            % Compute the control input in virtual space
            tV = Robot.LinearCtrlGain*(PG - RobotPositionTransformed);

            % Transform the input using the differential
            tV_transformed = RobotPositionTransformedD\transpose(tV);

            control_direction = atan2(tV_transformed(2), tV_transformed(1));
            control_unit = [cos(control_direction), sin(control_direction)];
            control_norm = sqrt(tV_transformed(1)^2+tV_transformed(2)^2);

            if control_norm >= Robot.LinearCmdMax
                control_norm = Robot.LinearCmdMax;
            end

            [i,j]

            quiver(point_to_examine(1),point_to_examine(2),control_norm*control_unit(1),control_norm*control_unit(2),'LineWidth',0.6,'Color','b','MaxHeadSize',0.6)

        end
    end



    drawnow;

    clear SemanticMap 


    % Save Figure Output
    if flagSaveFigure
        set(gcf, 'Renderer', 'painters')
        saveas(gcf, filename, 'svg')
        saveas(gcf, filename, 'epsc')
        print(gcf, '-opengl', '-dpng', '-r400', filename);
    end
end

























