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


% demoDiffeo.m
% A demonstration of a disk-shaped robot navigating towards a target
% location through obstacles
function [] = demoDiffeo(ScenarioID)
    %% Start with a clean simulation environment
    close all; % Close all figures
    clearvars -except ScenarioID; % Clear all variables
    rng(0); % Seeds the random number generator
    warning('off','all');
    %% Simulation Settings

    % Simulation Scenario:
    numSampleObstacle = 40; % Number of samples for representing obstacle surface
    numSampleLIDAR = 360; % Number of angular samples for LIDAR

    % Global Variables
    global Map;         % A convex polygonal environment populated with obstacles
    global KnownMap;    % Known semantic map to be used in the simulation
    global Robot;       % Robot Model
    global Goal;        % Goal Location
    Scenario = scenario(ScenarioID, numSampleObstacle, numSampleLIDAR); % Simulation Settings
    Option = option(ScenarioID); % Visualization Option

    %% Animate the resulting trajectory
    FrameRate = Option.FrameRate;
    dt = 1/FrameRate;
    % Multimedia output settings
    filename = sprintf('multimedia/demo_diffeo_v%d', ScenarioID);
    flagSaveVideo = 1; % Flag variable for avi video output
    flagSaveGif = 0; % Flag variable for gif animation output
    flagSaveFigure = 1; % Flag variable for saving figure
    flagDebug = 0; % Flag variables for debugging

    global pathTransformed; % Initializer for path in model space
    global pathGoalTransformed % Initializer for goal position array in model space

    figure
    [ax1,ax2,ax3,ax4] = plotworkspace(ScenarioID); % Plot workspace

    % Save Video/Animation Outputs
    if (flagSaveGif || flagSaveVideo)
        frame = getframe(gcf);
        if flagSaveVideo
           if strcmp(computer,'GLNXA64')
               handleVideoWriter = VideoWriter([filename '.avi'], 'Motion JPEG AVI');
           elseif strcmp(computer,'MACI64')
               handleVideoWriter = VideoWriter([filename '.mp4'], 'MPEG-4');
           end
           handleVideoWriter.FrameRate = FrameRate;
           open(handleVideoWriter);
           for ct = 1:FrameRate
               writeVideo(handleVideoWriter, frame);
           end
        end
        if flagSaveGif        
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            imwrite(imind, cm, [filename '.gif'], 'gif', 'DelayTime', 1/FrameRate, 'Loopcount',inf);
            for ct = 1:(FrameRate-1)
                imwrite(imind, cm, [filename '.gif'], 'gif','WriteMode','append');
            end
        end
    end

    for cs =  1: size(Scenario.Start,1)
        clear Map Robot Goal KnownMap
        global Map Robot Goal KnownMap
        % Reset the Map
        Map = Scenario.Map;
        Robot = Scenario.Robot;
        Goal = Scenario.Goal;

        % Initialize semantic map to be used in the simulation
        [Fx,Fy] = cvxpolyerode(Scenario.Map.Boundary(:,1), Scenario.Map.Boundary(:,2), Robot.Radius);
        KnownMap.Boundary = [Fx,Fy];
        KnownMap.SeenObstacles = Scenario.InitiallyKnown;
        
        % Find the union of all known obstacles
        poly_union = polyshape();
        for j=1:length(KnownMap.SeenObstacles)
            poly_union = union(poly_union, polyshape(Map.Obstacle{KnownMap.SeenObstacles(j)}.DilatedPoints));
        end

        % Split the result up to different polygons to find the familiar obstacles
        idx = all(isnan(poly_union.Vertices),2);
        idy = 1+cumsum(idx);
        idz = 1:size(poly_union.Vertices,1);
        KnownMap.FamiliarObstacles = accumarray(idy(~idx),idz(~idx),[],@(r){poly_union.Vertices(r,:)});
        
        % Create diffeomorphism parameters structure
        DiffeoParameters = py.dict();
        DiffeoParameters{'p'} = Map.DiffeoParams.p;
        DiffeoParameters{'epsilon'} = Map.DiffeoParams.epsilon;
        DiffeoParameters{'varepsilon'} = Map.DiffeoParams.varepsilon;
        DiffeoParameters{'mu_1'} = Map.DiffeoParams.mu1;
        DiffeoParameters{'mu_2'} = Map.DiffeoParams.mu2;
        DiffeoParameters{'workspace'} = py.numpy.array(py.numpy.round([Map.Boundary ; Map.Boundary(1,:)] + [Robot.Radius Robot.Radius ; Robot.Radius -Robot.Radius ; -Robot.Radius -Robot.Radius ; -Robot.Radius Robot.Radius ; Robot.Radius Robot.Radius],pyargs('decimals',int32(8))));
        
        % Calculate resulting trees
        for j=1:numel(KnownMap.FamiliarObstacles)
            KnownMap.FamiliarObstacles{j} = [KnownMap.FamiliarObstacles{j} ; KnownMap.FamiliarObstacles{j}(1,:)];
            [xout, yout] = poly2ccw(KnownMap.FamiliarObstacles{j}(:,1), KnownMap.FamiliarObstacles{j}(:,2));
            KnownMap.FamiliarObstacles{j} = [xout,yout];
            KnownMap.FamiliarObstaclesTrees{j} = py.reactive_planner_lib.diffeoTreeConvex(py.numpy.array(py.numpy.round(KnownMap.FamiliarObstacles{j},pyargs('decimals',int32(8)))), DiffeoParameters);
        end

        % Find the start configuration and initialize the LIDAR
        Start= Scenario.Start(cs,:); % Start Configuration
        R = readLIDAR2D(Start, Robot.Camera, Map);

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
        SemanticMap.SeenObstacles = Scenario.InitiallyKnown;
        
        % Find the union of all known obstacles
        poly_union = polyshape();
        for j=1:length(SemanticMap.SeenObstacles)
            poly_union = union(poly_union, polyshape(Map.Obstacle{SemanticMap.SeenObstacles(j)}.DilatedPoints));
        end
        
        % Split the result up to different polygons to find the familiar obstacles
        idx = all(isnan(poly_union.Vertices),2);
        idy = 1+cumsum(idx);
        idz = 1:size(poly_union.Vertices,1);
        SemanticMap.FamiliarObstacles = accumarray(idy(~idx),idz(~idx),[],@(r){poly_union.Vertices(r,:)});
        
        % Calculate resulting trees
        for j=1:numel(SemanticMap.FamiliarObstacles)
            SemanticMap.FamiliarObstacles{j} = [SemanticMap.FamiliarObstacles{j} ; SemanticMap.FamiliarObstacles{j}(1,:)];
            [xout, yout] = poly2ccw(SemanticMap.FamiliarObstacles{j}(:,1), SemanticMap.FamiliarObstacles{j}(:,2));
            SemanticMap.FamiliarObstacles{j} = [xout,yout];
            SemanticMap.FamiliarObstaclesTrees{j} = py.reactive_planner_lib.diffeoTreeConvex(py.numpy.array(py.numpy.round(SemanticMap.FamiliarObstacles{j},pyargs('decimals',int32(8)))), DiffeoParameters);
        end


        % Plot the physical workspace
        axes(ax1)
        hold on;

        % Plot goal
        hGoal = patch('XData', Goal(1)+Robot.Polygon(:,1), 'YData', Goal(2)+Robot.Polygon(:,2), Option.Goal{:});

        % Plot start
        hRobotStart = patch('XData', Start(1)+Robot.Polygon(:,1), 'YData', Start(2)+Robot.Polygon(:,2), Option.Start{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLStart = patch('XData', Start(1)+Robot.Radius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.Radius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRStart = patch('XData', Start(1)-Robot.Radius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.Radius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Plot the robot and the path
        hPath = plot(Start(1), Start(2), Option.Path{:});
        hRobot = patch('XData', Start(1)+Robot.Polygon(:,1), 'YData', Start(2)+Robot.Polygon(:,2), Option.Robot{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelL = patch('XData', Start(1)+Robot.Radius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.Radius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelR = patch('XData', Start(1)-Robot.Radius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.Radius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Initialize the field of view polygon
        polyFOV = polyshape([Start(1)+Robot.Camera.Range*cos(Robot.Camera.Angle)],[Start(2)+Robot.Camera.Range*sin(Robot.Camera.Angle)]);
        polyWorkspace = polyshape(MapCopy.Boundary(:,1),MapCopy.Boundary(:,2));
        polyFOV = intersect(polyFOV,polyWorkspace);
        hFOV = patch('XData', polyFOV.Vertices(:,1), 'YData', polyFOV.Vertices(:,2), Option.FOV{:});

        % Initialize discovered patches
        for i=1:numel(MapCopy.Obstacle)
            if (any(SemanticMap.SeenObstacles == i))
                hDiscovered(i) = patch('XData', MapCopy.Obstacle{i}.Points(:,1), 'YData', MapCopy.Obstacle{i}.Points(:,2), Option.DiscoveredObstacle{:});
            else
                hDiscovered(i) = patch('XData', [], 'YData', [], Option.DiscoveredObstacle{:});
            end
        end

        % Set the plot ordering
        uistack(hFOV, 'top')
        uistack(hGoal, 'top')
        uistack(hPath, 'top')
        uistack(hRobot, 'top')
        uistack(hRobotStart,'top')
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            uistack(hWheelL, 'top')
            uistack(hWheelR, 'top')
            uistack(hWheelLStart, 'top')
            uistack(hWheelRStart, 'top')
        end




        % Plot the semantic workspace
        axes(ax2)
        hold on;

        % Plot goal
        hGoalSemantic = patch('XData', Goal(1)+Robot.SmallPolygon(:,1), 'YData', Goal(2)+Robot.SmallPolygon(:,2), Option.Goal{:});

        % Plot start
        hRobotStartSemantic = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Start{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLStartSemantic = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRStartSemantic = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Plot the robot and the path
        hPathSemantic = plot(Start(1), Start(2), Option.Path{:});
        hRobotSemantic = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Robot{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLSemantic = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRSemantic = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Initialize discovered patch handles
        for i=1:numel(MapCopy.Obstacle)
            hDiscoveredSemanticFamiliar(i) = patch('XData', [], 'YData', [], Option.SemanticObstacle{:});
            hDiscoveredSemanticUnknown(i) = patch('XData', [], 'YData', [], Option.SemanticObstacle{:});
        end

        % Set the plot ordering
        uistack(hGoalSemantic, 'top')
        uistack(hPathSemantic, 'top')
        uistack(hRobotSemantic, 'top')
        uistack(hRobotStartSemantic,'top')
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            uistack(hWheelLSemantic, 'top')
            uistack(hWheelRSemantic, 'top')
            uistack(hWheelLStartSemantic, 'top')
            uistack(hWheelRStartSemantic, 'top')
        end




        % Plot the mapped workspace
        axes(ax3)
        hold on;

        % Plot goal
        hGoalMapped = patch('XData', Goal(1)+Robot.SmallPolygon(:,1), 'YData', Goal(2)+Robot.SmallPolygon(:,2), Option.Goal{:});

        % Plot start
        hRobotStartMapped = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Start{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLStartMapped = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRStartMapped = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Plot the robot and the path
        hPathMapped = plot(Start(1), Start(2), Option.Path{:});
        hRobotMapped = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Robot{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLMapped = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRMapped = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Initialize discovered patch handles
        for i=1:numel(MapCopy.Obstacle)
            hDiscoveredMappedFamiliar(i) = patch('XData', [], 'YData', [], Option.MappedObstacle{:});
            hDiscoveredMappedUnknown(i) = patch('XData', [], 'YData', [], Option.MappedObstacle{:});
        end

        % Set the plot ordering
        uistack(hGoalMapped, 'top')
        uistack(hPathMapped, 'top')
        uistack(hRobotMapped, 'top')
        uistack(hRobotStartMapped,'top')
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            uistack(hWheelLMapped, 'top')
            uistack(hWheelRMapped, 'top')
            uistack(hWheelLStartMapped, 'top')
            uistack(hWheelRStartMapped, 'top')
        end




        % Plot the model workspace
        axes(ax4)
        hold on;

        % Plot goal
        hGoalTransformed = patch('XData', Goal(1)+Robot.SmallPolygon(:,1), 'YData', Goal(2)+Robot.SmallPolygon(:,2), Option.Goal{:});

        % Plot start
        hRobotStartTransformed = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Robot{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLStartTransformed = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRStartTransformed = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Plot the robot and the path
        hPathTransformed = plot(Start(1), Start(2), Option.Path{:});
        hRobotTransformed = patch('XData', Start(1)+Robot.SmallPolygon(:,1), 'YData', Start(2)+Robot.SmallPolygon(:,2), Option.Robot{:});
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            hWheelLTransformed = patch('XData', Start(1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
            hWheelRTransformed = patch('XData', Start(1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', Start(2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
        end

        % Initialize free space and projected goal handles
        hLF = patch('XData', [], 'YData', [], Option.LocalFreespace{:});
        hPG = scatter([], [], [], Option.LocalGoal{:});

        % Initialize model obstacle handles
        for i=1:numel(MapCopy.Obstacle)
            hModelObstacle(i) = patch('XData', [], 'YData', [], Option.DiscoveredObstacle{:});
        end

        % Set the plot ordering
        uistack(hLF, 'top')
        uistack(hGoalTransformed, 'top')
        uistack(hPG, 'top')
        uistack(hPathTransformed, 'top')
        uistack(hRobotTransformed, 'top')
        uistack(hRobotStartTransformed,'top')
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            uistack(hWheelLTransformed, 'top')
            uistack(hWheelRTransformed, 'top')
            uistack(hWheelLStartTransformed, 'top')
            uistack(hWheelRStartTransformed, 'top')
        end




        % Numerically solve the navigation dynamics
        tspan = Scenario.ODE.tspan;
        x0 = Start(:);
        if Scenario.Type(cs) == 0
            [ode, out] = getHandles_particle_vf();
            odeoption = odeset(Scenario.ODE.option, 'Events', @ev, 'OutputFcn', out);
        elseif Scenario.Type(cs) == 1
            [ode, out] = getHandles_unicycle_vf();
            odeoption = odeset(Scenario.ODE.option, 'Events', @ev, 'OutputFcn', out);
        elseif Scenario.Type(cs) == 2
            [ode, out] = getHandles_particle_diffeo_polygon_vf();
            odeoption = odeset(Scenario.ODE.option, 'Events', @ev, 'OutputFcn', out);
        elseif Scenario.Type(cs) == 3
            [ode, out] = getHandles_unicycle_diffeo_polygon_vf();
            odeoption = odeset(Scenario.ODE.option, 'Events', @ev, 'OutputFcn', out);
        end
        [T, X, TE, XE, IE] = ode45(ode, tspan, x0, odeoption);
        filename_mat = sprintf('multimedia/demo_diffeo_mat_v%d_trial%d.mat', ScenarioID, cs);
        save(filename_mat,'T','X','Map','Robot','Goal','pathTransformed','pathGoalTransformed');
%         filename_mat = sprintf('multimedia/demo_diffeo_mat_v%d_trial%d.mat', ScenarioID, cs);
%         load(filename_mat);

        % Post-process transformed paths to make sure they have the same length
        x_transformed = pathTransformed(:,1);
        y_transformed = pathTransformed(:,2);
        angle_transformed = pathTransformed(:,3);
        len_transformed = 1:length(pathTransformed);
        len_transformed_final = linspace(len_transformed(1),len_transformed(end),length(X));
        x_transformed_final = interp1(len_transformed, x_transformed, len_transformed_final);
        y_transformed_final = interp1(len_transformed, y_transformed, len_transformed_final);
        angle_transformed_final = interp1(len_transformed, angle_transformed, len_transformed_final);
        pathTransformed = transpose([x_transformed_final ; y_transformed_final ; angle_transformed_final]);

        x_goal_transformed = pathGoalTransformed(:,1);
        y_goal_transformed = pathGoalTransformed(:,2);
        len_goal_transformed = 1:length(pathGoalTransformed);
        len_goal_transformed_final = linspace(len_goal_transformed(1),len_goal_transformed(end),length(X));
        x_goal_transformed_final = interp1(len_goal_transformed, x_goal_transformed, len_goal_transformed_final);
        y_goal_transformed_final = interp1(len_goal_transformed, y_goal_transformed, len_goal_transformed_final);
        pathGoalTransformed = transpose([x_goal_transformed_final ; y_goal_transformed_final]);

        % Animate resulting trajectory
        tpre = T(1);
        ct = 2;
        while (ct <= length(T))
           if ((T(ct) - tpre) >= dt)
               w = (tpre + dt - T(ct-1))/(T(ct)-T(ct-1));
               Xtemp = X(ct,:);
               RobotState = Xtemp';
               RobotPosition = [Xtemp(1), Xtemp(2)];
               RobotOrientation = Xtemp(3);
               tpre = tpre + dt;


               % Plot the physical workspace
               axes(ax1)
               hold on;

               % Plot the robot and the path
               set(hPath, 'XData', X(1:ct,1), 'YData', X(1:ct,2));
               set(hRobot, 'XData', RobotPosition(1) + Robot.Radius*cos(Robot.BodyAngle), 'YData', RobotPosition(2) + Robot.Radius*sin(Robot.BodyAngle));
               if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                  set(hWheelL, 'XData', RobotPosition(1) + Robot.Radius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) + Robot.Radius*sin(Robot.WheelAngle+RobotOrientation))
                  set(hWheelR, 'XData', RobotPosition(1) - Robot.Radius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) - Robot.Radius*sin(Robot.WheelAngle+RobotOrientation))
               end

               % Find the field of view polygon
               polyFOV = polyshape([RobotPosition(1)+1.01*Robot.Camera.Range*cos(Robot.Camera.Angle)],[RobotPosition(2)+1.01*Robot.Camera.Range*sin(Robot.Camera.Angle)]);
               polyWorkspace = polyshape(MapCopy.Boundary(:,1),MapCopy.Boundary(:,2));
               polyFOV = intersect(polyFOV,polyWorkspace);
               set(hFOV, 'XData', polyFOV.Vertices(:,1), 'YData', polyFOV.Vertices(:,2), Option.FOV{:});

               % Initialize list of unknown obstacles
               UnknownObstacles{:} = [];

               % Check collisions with any of the obstacles
               for i=1:numel(MapCopy.Obstacle)
                   % If this particular obstacle has already been seen, continue
                   if (any(SemanticMap.SeenObstacles == i))
                       continue;
                   else
                       % Find the intersection of the obstacle under consideration with the FOV
                       polyObstacle = polyshape(MapCopy.Obstacle{i}.Points(:,1),MapCopy.Obstacle{i}.Points(:,2));
                       polyIntersect = intersect(polyFOV,polyObstacle);

                       % If the FOV does not contain this obstacle, continue
                       if ~isempty(polyIntersect.Vertices)
                           % If the FOV contains this obstacle, check whether it is known or unknown
                           if (MapCopy.Obstacle{i}.Type ~= 0)
                               % Add the obstacle to the list of seen obstacles
                               SemanticMap.SeenObstacles = [SemanticMap.SeenObstacles, i];
                               set(hDiscovered(i), 'XData', MapCopy.Obstacle{i}.Points(:,1), 'YData', MapCopy.Obstacle{i}.Points(:,2), Option.DiscoveredObstacle{:});

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
                           elseif (MapCopy.Obstacle{i}.Type == 0)
                               % Dilate the obstacles and add only the visible portion in the map
                               polyBuffer = polybuffer(polyIntersect,Robot.Radius,'JointType','round');
                               x2 = flipud(polyBuffer.Vertices(:,1)); x2 = [x2;x2(1)];
                               y2 = flipud(polyBuffer.Vertices(:,2)); y2 = [y2;y2(1)];
                               UnknownObstacles{end} = [x2,y2];
                               UnknownObstacles{end+1} = [];
                               set(hDiscovered(i), 'XData', polyIntersect.Vertices(:,1), 'YData', polyIntersect.Vertices(:,2), Option.DiscoveredObstacle{:});
                           end
                       end
                   end
               end
               UnknownObstacles(end) = [];


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

               % Find the diffeomorphism at current position
               RobotPositionTransformed = [pathTransformed(ct,1), pathTransformed(ct,2)];
               RobotOrientationTransformed = pathTransformed(ct,3);


               % Construct a virtual LIDAR with 2*pi span to be used in the model space
               LIDAR.Range = Robot.Camera.LidarRange;
               LIDAR.Infinity = 20;
               LIDAR.MinAngle = -pi;
               LIDAR.MaxAngle = pi;
               LIDAR.NumSample = Robot.Camera.NumSample;
               LIDAR.Resolution = (LIDAR.MaxAngle - LIDAR.MinAngle)/(LIDAR.NumSample - 1);
               LIDAR.Angle = linspace(LIDAR.MinAngle, LIDAR.MaxAngle, LIDAR.NumSample);

               % Read LIDAR data and compute projected goal in model space; the robot
               % radius can be zero because we have already dilated the obstacles
               R = readLIDAR2D([RobotPositionTransformed, RobotOrientationTransformed], LIDAR, TemporaryMapModel);
               LF = localfreespaceLIDAR2D([RobotPositionTransformed, RobotOrientationTransformed], R, LIDAR, 0);
               PG = projgoalLIDAR2D([RobotPositionTransformed, RobotOrientationTransformed], R, LIDAR, 0, [pathGoalTransformed(ct,1),pathGoalTransformed(ct,2)]);




               % Plot the semantic workspace
               axes(ax2)
               hold on;

               % Plot the robot and the path
               set(hPathSemantic, 'XData', X(1:ct,1), 'YData', X(1:ct,2));
               set(hRobotSemantic, 'XData', RobotPosition(1) + Robot.SmallRadius*cos(Robot.BodyAngle), 'YData', RobotPosition(2) + Robot.SmallRadius*sin(Robot.BodyAngle));
               if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                  set(hWheelLSemantic, 'XData', RobotPosition(1) + Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) + Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientation))
                  set(hWheelRSemantic, 'XData', RobotPosition(1) - Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) - Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientation))
               end

               % Plot the discovered obstacles
               for i=1:length(SemanticMap.SeenObstacles)
                   set(hDiscoveredSemanticFamiliar(i), 'XData', MapCopy.Obstacle{SemanticMap.SeenObstacles(i)}.DilatedPoints(:,1), 'YData', MapCopy.Obstacle{SemanticMap.SeenObstacles(i)}.DilatedPoints(:,2), Option.SemanticObstacle{:});
               end
               for i=(length(SemanticMap.SeenObstacles)+1):numel(MapCopy.Obstacle)
                   set(hDiscoveredSemanticFamiliar(i), 'XData', [], 'YData', [], Option.SemanticObstacle{:});
               end

               for i=1:numel(UnknownObstacles)
                   set(hDiscoveredSemanticUnknown(i), 'XData', UnknownObstacles{i}(:,1), 'YData', UnknownObstacles{i}(:,2), Option.SemanticObstacle{:});
               end
               for i=(numel(UnknownObstacles)+1):numel(MapCopy.Obstacle)
                   set(hDiscoveredSemanticUnknown(i), 'XData', [], 'YData', [], Option.SemanticObstacle{:});
               end




               % Plot the mapped workspace
               axes(ax3)
               hold on;

               % Plot the robot and the path
               set(hPathMapped, 'XData', X(1:ct,1), 'YData', X(1:ct,2));
               set(hRobotMapped, 'XData', RobotPosition(1) + Robot.SmallRadius*cos(Robot.BodyAngle), 'YData', RobotPosition(2) + Robot.SmallRadius*sin(Robot.BodyAngle));
               if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                  set(hWheelLMapped, 'XData', RobotPosition(1) + Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) + Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientation))
                  set(hWheelRMapped, 'XData', RobotPosition(1) - Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) - Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientation))
               end

               % Plot the discovered obstacles
               for i=1:numel(SemanticMap.FamiliarObstacles)
                   set(hDiscoveredMappedFamiliar(i), 'XData', SemanticMap.FamiliarObstacles{i}(:,1), 'YData', SemanticMap.FamiliarObstacles{i}(:,2), Option.MappedObstacle{:});
               end
               for i=(numel(SemanticMap.FamiliarObstacles)+1):numel(MapCopy.Obstacle)
                   set(hDiscoveredMappedFamiliar(i), 'XData', [], 'YData', [], Option.MappedObstacle{:});
               end

               for i=1:numel(UnknownObstacles)
                   set(hDiscoveredMappedUnknown(i), 'XData', UnknownObstacles{i}(:,1), 'YData', UnknownObstacles{i}(:,2), Option.MappedObstacle{:});
               end
               for i=(numel(UnknownObstacles)+1):numel(MapCopy.Obstacle)
                   set(hDiscoveredMappedUnknown(i), 'XData', [], 'YData', [], Option.MappedObstacle{:});
               end




               % Plot the model workspace
               axes(ax4)
               hold on;

               % Plot the robot and the path
               set(hRobotStartTransformed, 'XData', pathTransformed(1,1)+Robot.SmallPolygon(:,1), 'YData', pathTransformed(1,2)+Robot.SmallPolygon(:,2), Option.Robot{:});
               if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                   set(hWheelLStartTransformed, 'XData', pathTransformed(1,1)+Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', pathTransformed(1,2)+Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
                   set(hWheelRStartTransformed, 'XData', pathTransformed(1,1)-Robot.SmallRadius*cos(Robot.WheelAngle+Start(3)), 'YData', pathTransformed(1,2)-Robot.SmallRadius*sin(Robot.WheelAngle+Start(3)), Option.Wheel{:});
               end
               set(hPathTransformed, 'XData', pathTransformed(1:ct,1), 'YData', pathTransformed(1:ct,2));
               set(hRobotTransformed, 'XData', RobotPositionTransformed(1)+Robot.SmallPolygon(:,1), 'YData', RobotPositionTransformed(2)+Robot.SmallPolygon(:,2), Option.Robot{:});
               % Plot the wheels of the unicycle if necessary
               if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                  set(hWheelLTransformed, 'XData', RobotPositionTransformed(1) + Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientationTransformed), 'YData',RobotPositionTransformed(2) + Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientationTransformed))
                  set(hWheelRTransformed, 'XData', RobotPositionTransformed(1) - Robot.SmallRadius*cos(Robot.WheelAngle+RobotOrientationTransformed), 'YData',RobotPositionTransformed(2) - Robot.SmallRadius*sin(Robot.WheelAngle+RobotOrientationTransformed))
               end

               % Plot obstacles
               for i=1:numel(TemporaryMapModel.Obstacle)
                   set(hModelObstacle(i), 'XData', TemporaryMapModel.Obstacle{i}.Points(:,1), 'YData', TemporaryMapModel.Obstacle{i}.Points(:,2), Option.DiscoveredObstacle{:});
               end
               for i=(numel(TemporaryMapModel.Obstacle)+1):numel(MapCopy.Obstacle)
                   set(hModelObstacle(i), 'XData', [], 'YData', [], Option.DiscoveredObstacle{:});
               end

               % Plot local free space and the local goal
               set(hLF, 'XData', LF(:,1), 'YData', LF(:,2), Option.LocalFreespace{:});
               set(hPG, 'XData', PG(1), 'YData', PG(2));

               % Plot goal
               set(hGoalTransformed, 'XData', pathGoalTransformed(ct,1)+Robot.SmallPolygon(:,1), 'YData', pathGoalTransformed(ct,2)+Robot.SmallPolygon(:,2));




               drawnow;
               if flagSaveVideo || flagSaveGif
                    frame = getframe(gcf);    
                    if flagSaveVideo
                         writeVideo(handleVideoWriter, frame);
                    end
                    if flagSaveGif
                        im = frame2im(frame);
                        [imind,cm] = rgb2ind(im,256);
                        for ct = 1:2*FrameRate
                            imwrite(imind, cm, [filename '.gif'], 'gif','WriteMode','append');
                        end
                    end
               end
               pause(dt);
               clear UnknownObstacles TemporaryMapModel
           else
               ct = ct + 1;
           end
        end

        delete(hRobot);
        delete(hRobotSemantic);
        delete(hRobotMapped);
        delete(hRobotTransformed);
        delete(hFOV);
        delete(hLF);
        delete(hPG);
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            delete(hWheelL);
            delete(hWheelR);
            delete(hWheelLSemantic);
            delete(hWheelRSemantic);
            delete(hWheelLMapped);
            delete(hWheelRMapped);
            delete(hWheelLTransformed);
            delete(hWheelRTransformed);
        end
        drawnow;
        % Comment these if you want to visualize the path in all the layers
        delete(hPathSemantic);
        delete(hRobotStartSemantic);
        delete(hPathMapped);
        delete(hRobotStartMapped);
        delete(hPathTransformed);
        delete(hRobotStartTransformed);
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            delete(hWheelLStartSemantic);
            delete(hWheelRStartSemantic);
            delete(hWheelLStartMapped);
            delete(hWheelRStartMapped);
            delete(hWheelLStartTransformed);
            delete(hWheelRStartTransformed);
        end
        for i=1:numel(MapCopy.Obstacle)
            if ~(any(Scenario.InitiallyKnown == i))
                delete(hDiscovered(i));
            end
        end
        delete(hDiscoveredSemanticFamiliar(:));
        delete(hDiscoveredSemanticUnknown(:));
        delete(hDiscoveredMappedFamiliar(:));
        delete(hDiscoveredMappedUnknown(:));
        delete(hModelObstacle(:));

        clear SemanticMap 
    end


    % Save Video/Animation Outputs
    if (flagSaveGif || flagSaveVideo)
        frame = getframe(gcf);
        if flagSaveVideo
           for ct = 1:2*FrameRate
               writeVideo(handleVideoWriter, frame);
           end
        end
        if flagSaveGif
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            for ct = 1:2*FrameRate
                imwrite(imind, cm, [filename '.gif'], 'gif','WriteMode','append');
            end
        end
    end
    % Save Video Output 
    if flagSaveVideo
        close(handleVideoWriter);
    end
    % Save Figure Output
    if flagSaveFigure
        set(gcf, 'Renderer', 'painters')
        saveas(gcf, filename, 'svg')
        saveas(gcf, filename, 'epsc')
        print(gcf, '-opengl', '-dpng', '-r400', filename);
    end
end



