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


% demoRRT.m
% A demonstration of a disk-shaped robot navigating towards a target
% location through obstacles using RRT
function [] = demoRRT(ScenarioID)
    %% Start with a clean simulation environment
    close all; % Close all figures
    clear global;
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
    global Option;      % Plot options
    global ax;          % Defined axes
    
    % RRT variables
    global node_pos parent children edge_wt temp_edge_wt queue cost_goal neighbours line_handles rhs o_nodes discovered_obstacles;
    global node_pos_original parent_original children_original edge_wt_original temp_edge_wt_original queue_original cost_goal_original neighbours_original line_handles_original rhs_original;
    global path_to_follow path_to_follow_discretized goal_handles start_idx delta curr_node obstacle_cost;
    
    % Load scenario and plot options
    Scenario = scenario(ScenarioID, numSampleObstacle, numSampleLIDAR); % Simulation Settings
    Option = option(ScenarioID); % Visualization Option

    %% Animate the resulting trajectory
    FrameRate = Option.FrameRate;
    % Multimedia output settings
    filename = sprintf('multimedia/demo_RRT_v%d', ScenarioID);
    flagSaveVideo = 1; % Flag variable for avi video output
    flagSaveGif = 0; % Flag variable for gif animation output
    flagSaveFigure = 1; % Flag variable for saving figure
    flagDebug = 0; % Flag variables for debugging

    figure
    ax = plotworkspace_RRT(ScenarioID); % Plot workspace

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
        
        clear node_pos parent children edge_wt temp_edge_wt queue cost_goal neighbours line_handles rhs o_nodes discovered_obstacles
        clear node_pos_original parent_original children_original edge_wt_original temp_edge_wt_original queue_original cost_goal_original neighbours_original line_handles_original rhs_original
        clear path_to_follow path_to_follow_discretized goal_handles start_idx delta curr_node obstacle_cost
        global node_pos parent children edge_wt temp_edge_wt queue cost_goal neighbours line_handles rhs o_nodes discovered_obstacles;
        global node_pos_original parent_original children_original edge_wt_original temp_edge_wt_original queue_original cost_goal_original neighbours_original line_handles_original rhs_original;
        global path_to_follow path_to_follow_discretized goal_handles start_idx delta curr_node obstacle_cost;
    
        
        % Reset the Map
        Map = Scenario.Map;
        Robot = Scenario.Robot;
        Goal = Scenario.Goal;
        
        % Initialize RRT variables - epsilon: sampling radius around node,
        % ball_radius: admissible radius to connect samples
        samples = Scenario.RRT.Samples;
        epsilon = Scenario.RRT.Epsilon;     delta = 0;                                 ball_radius = (Scenario.RRT.BallRadius)^2;
        node_pos = NaN(samples,2);          parent = NaN(samples,1);                   edge_wt = cell(samples,1);          
        neighbours = cell(samples,1);       cost_goal = Inf(samples,1);                rhs = Inf(samples,1);
        children = cell(samples,1);         goal_handles = gobjects(samples,1);        queue = Inf(samples,3);          
        rng(1);
        index_i = 1;                        choose_goal_node_at_i = floor(0.8*samples):50:samples;        goal_chosen = 0;
        node_pos(index_i,:) = Goal(1:2);    cost_goal(index_i) = 0;
        rhs(index_i) = 0;                   discovered_obstacles = [];
        pause_on = 0.001;                   line_handles = gobjects(samples,1);
        goalAchieved=0;
        

        % Initialize semantic map to be used in the simulation
        [Fx,Fy] = cvxpolyerode(Scenario.Map.Boundary(:,1), Scenario.Map.Boundary(:,2), Robot.Radius);
        KnownMap.Boundary = [Fx,Fy];
        KnownMap.SeenObstacles = [];
        KnownMap.FamiliarObstacles = [];

        % Find the start configuration and initialize the LIDAR
        Start= Scenario.Start(cs,:); % Start Configuration
        R = readLIDAR2D(Start, Robot.Camera, Map);

        % Initialize a map copy to be used locally for the animation
        MapCopy.Obstacle = Map.Obstacle;
        MapCopy.Boundary = Map.Boundary;
        MapCopy.KnownObstacle = Map.KnownObstacle;
        
        % Initialize the semantic map to be used for the animation locally
        SemanticMap.Boundary = KnownMap.Boundary;
        SemanticMap.SeenObstacles = [];
        SemanticMap.FamiliarObstacles = [];
        


        % Plot the physical workspace
        axes(ax)
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
        
        % Initialize free space and projected goal handles
        hLF = patch('XData', [], 'YData', [], Option.LocalFreespace{:});
        hPG = scatter([], [], [], Option.LocalGoal{:});

        % Initialize discovered patches
        for i=1:numel(MapCopy.Obstacle)
            hDiscovered(i) = patch('XData', [], 'YData', [], Option.DiscoveredObstacle{:});
        end
        
        % Find initial tree for RRT
        tic
        while index_i<samples
            index_i
        %   'index_i' is the Node number
            index_i = index_i+1;
            if ~goal_chosen && any(index_i == choose_goal_node_at_i) 
                rand_point = Start(1:2);
            else
                rand_point = [MapCopy.Boundary(1,1)+Robot.Radius, MapCopy.Boundary(1,2)+Robot.Radius] + [(MapCopy.Boundary(3,1)-Robot.Radius)-(MapCopy.Boundary(1,1)+Robot.Radius), (MapCopy.Boundary(3,2)-Robot.Radius)-(MapCopy.Boundary(1,2)+Robot.Radius)].*rand(1,2);
            end
        %   Calculate the distance of random point to all the nodes
            dist_points = pdist2(node_pos,rand_point,'squaredeuclidean');
            [sqDist,idx] = min(dist_points);

        %   Bring the node at a distance of epsilon near to the closest node
            n_point = node_pos(idx,:);
            n_dist = realsqrt(sqDist);
            min_dist = min(n_dist,epsilon); % If the node is closer than epsilon, we have the same distance
            rand_point = n_point + (rand_point - n_point)*min_dist/n_dist;
            node_pos(index_i,:) = rand_point;

        %   After bringing the node closer, now calculate the distances to all the nodes again
            all_edge_wt = pdist2(node_pos,rand_point,'squaredeuclidean');
            n_node_idx = all_edge_wt < ball_radius & all_edge_wt > 0 ;

        %   Select the nodes that are within the ball radius. These become the neighbours. The 'n_' represents neighbours 
            n_nodes = find(n_node_idx);
            n_edge_wt = realsqrt(all_edge_wt(n_node_idx));
            n_rhs = rhs(n_node_idx);

        %   Find the node with the smallest 'rhs'
            n_total_cost = n_edge_wt + n_rhs;
            [p_cost,p_idx] = min(n_total_cost);

        %   Initialize the node properties
            parent(index_i) = n_nodes(p_idx);
            rhs(index_i) = p_cost;
            neighbours(index_i) = {n_nodes};
            edge_wt(index_i) = {n_edge_wt};

        %   Now we have to tell the random points' neighbours that random point is
        %   now a neighbour, also add the corresponding edge weight
            for j = 1:length(n_nodes)
                neighbours(n_nodes(j)) = {[neighbours{n_nodes(j)};index_i]};
                edge_wt(n_nodes(j)) = {[edge_wt{n_nodes(j)};n_edge_wt(j)]};
            end

        %   Plot line from random point to its parent
            pause(pause_on);
%             plot(rand_point(1),rand_point(2),Option.RRTpoint{:});
            handle = line([node_pos(parent(index_i),1) node_pos(index_i,1)],[node_pos(parent(index_i),2) node_pos(index_i,2)], Option.RRTline{:});
            line_handles(index_i) = handle;
            uistack(line_handles(index_i),'top')

        %   From now on it is from RRTx Paper 
            rewireNeighbours(index_i);
            reduceInconsistency();

        %   If the goal has been reached, plot the goal line
            if rand_point == Start(1:2) 
                start_idx = index_i;
                plotFrom(start_idx);
                goalAchieved = 1;
                goal_chosen = 1;
                previous_goal_cost = cost_goal(start_idx);
            end

        %   If the cost to goal has decreased,then replot the goal line
            if  goalAchieved && (cost_goal(start_idx)<previous_goal_cost)
                delete(goal_handles);
                plotFrom(start_idx); 
                previous_goal_cost = cost_goal(start_idx);
            end
                        
        %   Draw tree expansion
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
        end
        toc
        
        % Destroy the remaining unused space - This is done for faster execution
        cost_goal(index_i+1:end) = [];               neighbours(index_i+1:end) = [];               edge_wt(index_i+1:end) = []; 
        line_handles(index_i+1:end) = [];            parent(index_i+1:end) = [];                   node_pos(index_i+1:end,:) = [];
        children(index_i+1:end)=[];                  goal_handles(index_i+1:end) = [];                  
        temp_edge_wt = edge_wt;                      temp_children = children;
        obstacle_cost = Inf;                         curr_node = start_idx;
        
        % Get all the children in a separate variable (cell)
        for k = 1:index_i  
            children(k) = {find(parent==k)};
        end
        
        % Initialize original variables
        node_pos_original = node_pos; parent_original = parent; children_original = children; edge_wt_original = edge_wt; temp_edge_wt_original = temp_edge_wt;
        queue_original = queue; cost_goal_original = cost_goal; neighbours_original = neighbours; line_handles_original = line_handles; rhs_original = rhs;
        
        % Update the path to follow
        % WE ASSUME THAT A PATH TO THE GOAL HAS ALREADY BEEN FOUND
        path_discretization = Scenario.RRT.PathDiscretization;
        for i=1:size(path_to_follow,1)-1
            dist = sqrt((path_to_follow(i,1)-path_to_follow(i+1,1))^2 + (path_to_follow(i,2)-path_to_follow(i+1,2))^2);
            num_samples = round(dist/path_discretization);
            path_gen_temp{i} = [path_to_follow(i,1) path_to_follow(i,2)];
            for j=1:num_samples
                path_gen_temp{i} = [path_gen_temp{i} ; path_to_follow(i,1)+(path_to_follow(i+1,1)-path_to_follow(i,1))*j/num_samples path_to_follow(i,2)+(path_to_follow(i+1,2)-path_to_follow(i,2))*j/num_samples];
            end
        end
        path_to_follow_discretized = path_gen_temp{1};
        for i=2:size(path_to_follow,1)-1
            path_to_follow_discretized = [path_to_follow_discretized ; [path_gen_temp{i}(:,1) path_gen_temp{i}(:,2)]];
        end

        % Set the plot ordering
        uistack(hFOV, 'top')
        uistack(hLF, 'top')
        uistack(hPG, 'top')
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
        
        
        
        % Initialize to solve navigation dynamics
        tspan = Scenario.ODE.tspan(2);
        t = 0.0;
        dt = 1e-1;
        t_last_rrt_update = 0.0;
        t_last_draw = -1.0;
        X = Start(:)';
        path = X;
        while (t < tspan) && ((X(1) - path_to_follow(end,1))^2 + (X(2) - path_to_follow(end,2))^2 > Robot.RelTol)
            % Unwrap state
            RobotPosition = [X(1), X(2)];
            RobotOrientation = X(3);
            RobotState = [RobotPosition, RobotOrientation];
            
            % Read LIDAR data and compute projected goals for linear and angular motion
            R = readLIDAR2D(RobotState, Robot.Camera, Map);
            LF = localfreespaceLIDAR2D(RobotState, R, Robot.Camera, Robot.Radius);
            
            % Find radius of minimum distance to the boundary
            dist_fsbd = min(R)-Robot.Radius;
            
            % OPTION 1: Find goal within the field of view
%             in_disk = (sqrt((path_to_follow_discretized(:,1)-RobotPosition(1)).^2+(path_to_follow_discretized(:,2)-RobotPosition(2)).^2) <= dist_fsbd);
%             goal_index = find(in_disk==1, 1, 'last');
%             temp_goal = path_to_follow_discretized(goal_index,:);
%             if pdist2(node_pos(curr_node,:),RobotPosition)^2 < Robot.RelTol
%                 curr_node = parent(curr_node);
%             end
            
            % OPTION 2: Get the next node and follow it
            if (pdist2(node_pos(curr_node,:),RobotPosition)^2 < 5.0*Robot.RelTol) && size(path_to_follow,1)>1
                curr_node = parent(curr_node);
            end
            temp_goal = node_pos(curr_node,:);
            
            % Compute the control inputs
            if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                % Compute projected goals
                [PGL, PGA1, PGA2] = projgoalLIDAR2Dunicycle(RobotState, R, Robot.Camera, Robot.Radius, temp_goal);
                PGA = (PGA1 + PGA2)/2;
                PG = PGA1;
                
                % Compute the control inputs
                tV = (PGL - RobotPosition) * [cos(RobotOrientation); sin(RobotOrientation)];
                tW1 = (PGA - RobotPosition) * [cos(RobotOrientation); sin(RobotOrientation)];
                tW2 = (PGA - RobotPosition) * [-sin(RobotOrientation); cos(RobotOrientation)];
                dV = Robot.LinearCtrlGain * tV;
                dW = Robot.AngularCtrlGain * atan2(tW2, tW1);
                dx = [dV*cos(RobotOrientation), dV*sin(RobotOrientation), dW];
            else
                % Compute projected goals
                PG = projgoalLIDAR2D(RobotState, R, Robot.Camera, Robot.Radius, temp_goal);
                
                % Compute the control inputs
                tV = Robot.LinearCtrlGain*(PG - RobotPosition);
                dx = [tV(1), tV(2), 0];
            end
            
            % Plot updates
            % Plot the robot and the path
            set(hPath, 'XData', path(1:end,1), 'YData', path(1:end,2));
            set(hRobot, 'XData', RobotPosition(1) + Robot.Radius*cos(Robot.BodyAngle), 'YData', RobotPosition(2) + Robot.Radius*sin(Robot.BodyAngle));
            if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
                set(hWheelL, 'XData', RobotPosition(1) + Robot.Radius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) + Robot.Radius*sin(Robot.WheelAngle+RobotOrientation))
                set(hWheelR, 'XData', RobotPosition(1) - Robot.Radius*cos(Robot.WheelAngle+RobotOrientation), 'YData',RobotPosition(2) - Robot.Radius*sin(Robot.WheelAngle+RobotOrientation))
            end
            
            % Find the field of view polygon
            polyFOV = polyshape([RobotPosition(1)+Robot.Camera.Range*cos(Robot.Camera.Angle)],[RobotPosition(2)+Robot.Camera.Range*sin(Robot.Camera.Angle)]);
            polyWorkspace = polyshape(MapCopy.Boundary(:,1),MapCopy.Boundary(:,2));
            polyFOV = intersect(polyFOV,polyWorkspace);
            set(hFOV, 'XData', polyFOV.Vertices(:,1), 'YData', polyFOV.Vertices(:,2), Option.FOV{:});
            
            % Plot local free space and the local goal
            set(hLF, 'XData', LF(:,1), 'YData', LF(:,2), Option.LocalFreespace{:});
            set(hPG, 'XData', PG(1), 'YData', PG(2));
            
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
            
            % Updates
            path = [path ; X];
            X = X + dx*dt;
            t = t+dt;
            
            % RRT update when necessary
            if (t-t_last_rrt_update > (1.0/Scenario.RRT.UpdateRate))
                % Revert to original RRT
%                 node_pos = node_pos_original; parent = parent_original; children = children_original; edge_wt = edge_wt_original; temp_edge_wt = temp_edge_wt_original;
%                 queue = queue_original; cost_goal = cost_goal_original; neighbours = neighbours_original; line_handles = line_handles_original; rhs = rhs_original; o_nodes = [];
        
                % Update list of obstacles
                obstacles{:} = [];
                for i=1:numel(SemanticMap.FamiliarObstacles)
                    obstacles{end} = SemanticMap.FamiliarObstacles{i};
                    obstacles{end+1} = [];
                end
                for i=1:numel(UnknownObstacles)
                    obstacles{end} = UnknownObstacles{i};
                    obstacles{end+1} = [];
                end
                obstacles(end) = [];
                
                % Find nodes hit by obstacles
                o_nodes = getObstacleNodes(obstacles);
                clear obstacles
                
                % Update
                r_dist = pdist2(node_pos,RobotPosition,'squaredeuclidean');
                in_range = find(r_dist<Robot.Camera.Range);
                updateObstacles(o_nodes);
                delete(goal_handles);
                plotFrom(curr_node);
                
                % Discretize generated path
                path_discretization = Scenario.RRT.PathDiscretization;
                for i=1:size(path_to_follow,1)-1
                    dist = sqrt((path_to_follow(i,1)-path_to_follow(i+1,1))^2 + (path_to_follow(i,2)-path_to_follow(i+1,2))^2);
                    num_samples = round(dist/path_discretization);
                    path_gen_temp{i} = [path_to_follow(i,1) path_to_follow(i,2)];
                    for j=1:num_samples
                        path_gen_temp{i} = [path_gen_temp{i} ; path_to_follow(i,1)+(path_to_follow(i+1,1)-path_to_follow(i,1))*j/num_samples path_to_follow(i,2)+(path_to_follow(i+1,2)-path_to_follow(i,2))*j/num_samples];
                    end
                end
                path_to_follow_discretized = path_gen_temp{1};
                for i=2:size(path_to_follow,1)-1
                    path_to_follow_discretized = [path_to_follow_discretized ; [path_gen_temp{i}(:,1) path_gen_temp{i}(:,2)]];
                end
                
                t_last_rrt_update = t;
            end
            
            % Plot
            if (t-t_last_draw) > 1/FrameRate
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
                pause(1/FrameRate);
                t_last_draw = t;
            end
            clear UnknownObstacles
        end        

        delete(hRobot);
        delete(hFOV);
        delete(hLF);
        delete(hPG);
        if (Scenario.Type(cs)==1) || (Scenario.Type(cs)==3)
            delete(hWheelL);
            delete(hWheelR);
        end
        drawnow;
        delete(hDiscovered(:));

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




function updateObstacles(nodes)

global o_nodes curr_node;

obstacle_inRange = intersect(nodes,o_nodes);

if any(obstacle_inRange)
    addNewObstacle(obstacle_inRange);
    propogateDescendants();
    verifyQueue(curr_node);
    reduceInconsistency_v2();
end

end

function addNewObstacle(nodes)

global neighbours obstacle_cost edge_wt parent temp_edge_wt;

    for i = 1:length(nodes)
        node = nodes(i);
        edge_wt(node) = {temp_edge_wt{node} + obstacle_cost};
        its_neighbours = neighbours{node};
        for j = 1:length(its_neighbours)
           neigh = its_neighbours(j);
           idx = find(neighbours{neigh}==node);
           t_edge = edge_wt{neigh};       
           t_edge(idx) = temp_edge_wt{neigh}(idx) + obstacle_cost;
           edge_wt(neigh) = {t_edge};
           if parent(neigh)==node
                verifyOrphan(neigh);
           end
        end
    end
    
    
end

function propogateDescendants()

global discovered_obstacles children neighbours parent cost_goal rhs line_handles;

nodes = discovered_obstacles;
all_children = [];
tic;
while ~isempty(nodes) && toc < 2
    nodes = unique(cell2mat(children(nodes)));  
    all_children = union(all_children,nodes);
end

discovered_obstacles = union(discovered_obstacles,all_children);

for i = 1:length(discovered_obstacles)
   spl_nodes = setdiff(neighbours{discovered_obstacles(i)},discovered_obstacles);
   if ~isempty(spl_nodes)
    cost_goal(spl_nodes) = Inf;
    verifyQueue(spl_nodes); 
   end
end

cost_goal(discovered_obstacles) = Inf;
rhs(discovered_obstacles) = Inf;
for i = 1:length(discovered_obstacles)
    its_parent = parent(discovered_obstacles(i));
     if ~isnan(its_parent)
         child = children{its_parent};
         child_less = child(child ~= discovered_obstacles(i));
         children(its_parent) = {child_less};
     end
end
for j=discovered_obstacles
    set(line_handles(j), 'XData', [], 'YData', []);
end
% delete(line_handles(discovered_obstacles));
parent(discovered_obstacles) = NaN;
discovered_obstacles = [];

end

function verifyOrphan(node)

global queue discovered_obstacles;

if ~isempty(find(queue(:,1)==node,1))
    queue(queue(:,1)==node,:) = Inf;
end
discovered_obstacles = union(discovered_obstacles,node);

end

function plotFrom(iter)
global node_pos parent goal_handles path_to_follow;
global Option
k = 1;
path_to_follow = [];
iter_temp = iter;
while ~isnan(parent(iter_temp))
    iter_temp = parent(iter_temp);
    if isnan(parent(iter_temp)) && iter_temp~=1
        path_to_follow = [path_to_follow ; node_pos(iter,1), node_pos(iter,2)];
        delete(goal_handles);
        return;
    end
end
    
while ~isnan(parent(iter))
    next_iter = parent(iter);
    path_to_follow = [path_to_follow ; node_pos(iter,1), node_pos(iter,2)];
    goal_handles(k) = line([node_pos(iter,1) node_pos(next_iter,1)],[node_pos(iter,2) node_pos(next_iter,2)],Option.RRTpath{:});
    iter = next_iter;
    k = k+1;
end
path_to_follow = [path_to_follow ; node_pos(iter,1), node_pos(iter,2)];
end

function verifyQueue(nodes)

global queue;

key = getKeys(nodes);
len = length(nodes);
for i=1:len
   idx = find(queue(:,1)==nodes(i)); 
    if ~isempty(idx)
        queue(idx,2:3) = key(i,:);
    else
        insertNodes(nodes(i))
    end
end

end

function insertNodes(nodes)

global queue;

key = getKeys(nodes);
queue(end-length(nodes)+1:end,:) = [nodes,key];
queue = sortrows(queue,[2 3 1]);

end

function key = getKeys(nodes)

global rhs cost_goal;

key = [min([cost_goal(nodes),rhs(nodes)],[],2), cost_goal(nodes)];

end

function top_value = popQueue()

global queue;

top_value = queue(1,:);
queue(1,:) = Inf;
queue = circshift(queue,-1);

end

function top_value = topQueue()

global queue;
top_value = queue(1);

end

function updateRHS(node)

global neighbours rhs edge_wt parent discovered_obstacles children;

neigh = neighbours{node};
edge = edge_wt{node};
banned_nodes = union(discovered_obstacles,neigh(parent(neigh)==node));
[~,idx] = setdiff(neigh,banned_nodes);
neigh = neigh(idx);
edge = edge(idx);

[updated_rhs,new_parent] = min(edge + rhs(neigh));
rhs(node) = updated_rhs;
parent(node) = neigh(new_parent);
children(neigh(new_parent)) = {unique([children{neigh(new_parent)};node])};
end

function rewireNeighbours(node)

global cost_goal rhs delta neighbours edge_wt parent line_handles node_pos children;
global Option

if cost_goal(node)-rhs(node) > delta || isnan(cost_goal(node)-rhs(node))
    neigh = neighbours{node};
    edge = edge_wt{node};
    idx = parent(neigh) ~= node;
    neigh = neigh(idx);
    edge = edge(idx);
    
    index = rhs(neigh) > edge + rhs(node);
    to_update = neigh(index);
    rhs(to_update) = edge(index) + rhs(node);
    parent(to_update) = node;
    children(node) = {unique([children{node};to_update])};
    for j = 1:length(to_update)
       set(line_handles(to_update(j)),'XData',[node_pos(to_update(j),1) node_pos(node,1)],'YData',[node_pos(to_update(j),2) node_pos(node,2)]);
    end
        
    further_index = (cost_goal(to_update) - rhs(to_update) > delta) | (isnan(cost_goal(to_update) - rhs(to_update)));
    verifyQueue(to_update(further_index));
    
end

end

function reduceInconsistency()
    
global queue cost_goal rhs delta;

while any(any(~isinf(queue)))
   top = popQueue();
   
   if cost_goal(top(1)) - rhs(top(1)) > delta || isnan(cost_goal(top(1)) - rhs(top(1)))
        updateRHS(top(1));
        rewireNeighbours(top(1));      
   end
   cost_goal(top(1)) = rhs(top(1));
   
end

end

function reduceInconsistency_v2()
    
global queue cost_goal rhs delta curr_node;

while any(any(~isinf(queue))) && (keyLess(topQueue(),curr_node) || rhs(curr_node)~=cost_goal(curr_node) || cost_goal(curr_node)==Inf)
   top = popQueue();
   
   if cost_goal(top(1)) - rhs(top(1)) > delta || isnan(cost_goal(top(1)) - rhs(top(1)))
        updateRHS(top(1));
        rewireNeighbours(top(1));      
   end
   cost_goal(top(1)) = rhs(top(1));
   
end

end

function value = keyLess(top,start)

top_key = getKeys(top);        start_key = getKeys(start);
value = top_key(1)<start_key(1) & top_key(2)<start_key(2);

end

function plotObstacles(obstacles)

[rows,~] = size(obstacles);
hold on;
for i = 1:rows
    rectangle('Position',obstacles(i,:),'FaceColor','r');
end
hold off;

end

function curr_ptr = plotIndication(curr_ptr)
global node_pos curr_node r_radius;
    hold on;
    curr_ptr(1) = plot(node_pos(curr_node,1),node_pos(curr_node,2),'g.','MarkerSize',20);
    curr_ptr(2) = viscircles(node_pos(curr_node,:),r_radius,'LineStyle','--','Color','r');
    hold off;
end

function o_nodes = getObstacleNodes(obstacles)

global node_pos;

o_nodes = [];
for i=1:numel(obstacles)
    obstacle_polygon = polyshape(obstacles{i});
    idx = isinterior(obstacle_polygon, node_pos(:,1), node_pos(:,2));
    o_nodes = union(o_nodes,find(idx));
end


end



