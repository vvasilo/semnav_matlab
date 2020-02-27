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


function [ode, out] = getHandles_unicycle_diffeo_polygon_vf()

    ode = @unicycle_diffeo_polygon_vf;
    out = @output_function;
    
    global pathTransformed
    global pathGoalTransformed
    
    RobotPositionTransformed = [];
    RobotOrientationTransformed = [];
    pathTransformed = [];
    GoalTransformed = [];
    pathGoalTransformed = [];
    
    function dy = unicycle_diffeo_polygon_vf(t,y)

        % Global Simulation Variables
        global Map; % Workspace Model
        global KnownMap; % Internal Semantic Map
        global Robot; % Robot Model
        global Goal; % Goal Location

        % Create diffeomorphism parameters structure
        DiffeoParameters = py.dict();
        DiffeoParameters{'p'} = Map.DiffeoParams.p;
        DiffeoParameters{'epsilon'} = Map.DiffeoParams.epsilon;
        DiffeoParameters{'varepsilon'} = Map.DiffeoParams.varepsilon;
        DiffeoParameters{'mu_1'} = Map.DiffeoParams.mu1;
        DiffeoParameters{'mu_2'} = Map.DiffeoParams.mu2;
        DiffeoParameters{'workspace'} = py.numpy.array(py.numpy.round([Map.Boundary ; Map.Boundary(1,:)] + [Robot.Radius Robot.Radius ; Robot.Radius -Robot.Radius ; -Robot.Radius -Robot.Radius ; -Robot.Radius Robot.Radius ; Robot.Radius Robot.Radius],pyargs('decimals',int32(8))));

        % Robot Configuration
        RobotState = y(:)';
        RobotPosition = [y(1), y(2)];
        RobotOrientation = y(3);

        % Build a field-of-view polygon based on the given camera model
        fov_polygon = [RobotPosition(1)+Robot.Camera.Range*cos(Robot.Camera.Angle) ; RobotPosition(2)+Robot.Camera.Range*sin(Robot.Camera.Angle)]';

        % Initialize list of unknown obstacles
        UnknownObstacles{:} = [];

        % Examine "collisions" of the FOV polygon with any of the obstacles given
        poly_fov = polyshape(fov_polygon(:,1),fov_polygon(:,2));
        poly_workspace = polyshape(KnownMap.Boundary(:,1),KnownMap.Boundary(:,2));
        poly_fov = intersect(poly_fov,poly_workspace);
        for i=1:numel(Map.Obstacle)
            % If this particular obstacle has already been seen, continue
            if (any(KnownMap.SeenObstacles == i))
                continue;
            else
                % Find the intersection of the obstacle under consideration with the FOV
                poly_obstacle = polyshape(Map.Obstacle{i}.Points(:,1),Map.Obstacle{i}.Points(:,2));
                poly_intersect = intersect(poly_fov,poly_obstacle);

                % If the FOV does not contain this obstacle, continue
                if ~isempty(poly_intersect.Vertices)
                    % If the FOV contains this obstacle, check whether it is known or unknown
                    if (Map.Obstacle{i}.Type ~=0)
                        % Add the obstacle to the list of seen obstacles
                        KnownMap.SeenObstacles = [KnownMap.SeenObstacles, i];

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
                        for j=1:numel(KnownMap.FamiliarObstacles)
                            [xout,yout] = poly2ccw(KnownMap.FamiliarObstacles{j}(:,1),KnownMap.FamiliarObstacles{j}(:,2));
                            KnownMap.FamiliarObstacles{j} = [xout,yout];
                        end

                        % Calculate resulting trees
                        for j=1:numel(KnownMap.FamiliarObstacles)
                            KnownMap.FamiliarObstacles{j} = [KnownMap.FamiliarObstacles{j} ; KnownMap.FamiliarObstacles{j}(1,:)];
                            [xout, yout] = poly2ccw(KnownMap.FamiliarObstacles{j}(:,1), KnownMap.FamiliarObstacles{j}(:,2));
                            KnownMap.FamiliarObstacles{j} = [xout,yout];
                            KnownMap.FamiliarObstaclesTrees{j} = py.reactive_planner_lib.diffeoTreeConvex(py.numpy.array(py.numpy.round(KnownMap.FamiliarObstacles{j},pyargs('decimals',int32(8)))), DiffeoParameters);
                        end
                    elseif (Map.Obstacle{i}.Type == 0)
                        % Dilate the obstacles and add only the visible portion in the map
                        poly_out = polybuffer(poly_intersect,Robot.Radius,'JointType','round');
                        x2 = flipud(poly_out.Vertices(:,1)); x2 = [x2;x2(1)];
                        y2 = flipud(poly_out.Vertices(:,2)); y2 = [y2;y2(1)];
                        UnknownObstacles{end} = [x2,y2];
                        UnknownObstacles{end+1} = [];
                    end
                end
            end
        end
        UnknownObstacles(end) = [];

        % Construct the model space based on the semantic map created above: the
        % model space will contain the equivalent spheres for the known obstacles
        % and the dilated portions of the unknown obstacles
        TemporaryMapModel.Boundary = KnownMap.Boundary;
        TemporaryMapModel.Obstacle{:} = [];
        for i=1:numel(KnownMap.FamiliarObstacles)
            TemporaryMapModel.Obstacle{end}.Points = obstacle2D_ball(double(KnownMap.FamiliarObstaclesTrees{i}{end}{'center'}), double(KnownMap.FamiliarObstaclesTrees{i}{end}{'radius'}), 100);
            TemporaryMapModel.Obstacle{end+1} = [];
        end
        for i=1:numel(UnknownObstacles)
            TemporaryMapModel.Obstacle{end}.Points = UnknownObstacles{i};
            TemporaryMapModel.Obstacle{end+1} = [];
        end
        TemporaryMapModel.Obstacle(end) = [];

        % Find the diffeomorphism and its jacobian at the robot position, along
        % with the necessary second derivatives - Also find the goal
        % position in the model space
        RobotPositionTransformed = RobotPosition;
        RobotPositionTransformedD = eye(2);
        RobotPositionTransformedDD = zeros(1,8);
        GoalTransformed = Goal;
        for i=1:numel(KnownMap.FamiliarObstacles)
            diffeo_result = py.reactive_planner_lib.polygonDiffeoConvex(py.numpy.array(RobotPositionTransformed), KnownMap.FamiliarObstaclesTrees{i}, DiffeoParameters);
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
                        
            diffeo_result = py.reactive_planner_lib.polygonDiffeoConvex(py.numpy.array(GoalTransformed), KnownMap.FamiliarObstaclesTrees{i}, DiffeoParameters);
            GoalTransformed = double(diffeo_result{1});
        end

        % Find alpha1, alpha2, beta1, beta2
        alpha1 = -(RobotPositionTransformedD(2,1)*cos(RobotOrientation)+RobotPositionTransformedD(2,2)*sin(RobotOrientation));
        beta1 = RobotPositionTransformedDD(1)*cos(RobotOrientation)^2+(RobotPositionTransformedDD(2)+RobotPositionTransformedDD(3))*sin(RobotOrientation)*cos(RobotOrientation)+RobotPositionTransformedDD(4)*sin(RobotOrientation)^2;
        alpha2 = RobotPositionTransformedD(1,1)*cos(RobotOrientation)+RobotPositionTransformedD(1,2)*sin(RobotOrientation);
        beta2 = RobotPositionTransformedDD(5)*cos(RobotOrientation)^2+(RobotPositionTransformedDD(6)+RobotPositionTransformedDD(7))*sin(RobotOrientation)*cos(RobotOrientation)+RobotPositionTransformedDD(8)*sin(RobotOrientation)^2;

        % Find robot orientation in the model space
        [V,D] = eig(RobotPositionTransformedD*RobotPositionTransformedD');
        dmax = max([D(1,1),D(2,2)]);
        dmin = min([D(1,1),D(2,2)]);
        e_vector = RobotPositionTransformedD * [cos(RobotOrientation) ; sin(RobotOrientation)];
        RobotOrientationTransformed = atan2(e_vector(2),e_vector(1));

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
        R = readLIDAR2D([RobotPositionTransformed, RobotOrientationTransformed], LIDAR, TemporaryMapModel);
        [PGL, PGA1, PGA2] = projgoalLIDAR2Dunicycle([RobotPositionTransformed, RobotOrientationTransformed], R, LIDAR, 0, GoalTransformed);
        PGA = PGA1;

        % Compute the basis for the virtual control inputs
        tV = (PGL - RobotPositionTransformed) * [cos(RobotOrientationTransformed); sin(RobotOrientationTransformed)];
        tW1 = (PGA - RobotPositionTransformed) * [cos(RobotOrientationTransformed); sin(RobotOrientationTransformed)];
        tW2 = (PGA - RobotPositionTransformed) * [-sin(RobotOrientationTransformed); cos(RobotOrientationTransformed)];

        % Compute the basis for transforming to actual control inputs
        e_norm = norm(e_vector);
        dksi_dpsi = det(RobotPositionTransformedD)/e_norm^2;
        DksiCosSin = (alpha1*beta1+alpha2*beta2)/e_norm^2;

        % Compute commands by accounting for limits
        LinearCtrlGain = min([Robot.LinearCtrlGain,Robot.LinearCmdMax*e_norm/abs(tV),0.5*Robot.AngularCmdMax*dksi_dpsi*e_norm/(abs(tV*DksiCosSin))]);
        AngularCtrlGain = min([Robot.AngularCtrlGain,0.5*(Robot.AngularCmdMax*dksi_dpsi)/(abs(atan2(tW2,tW1)))]);

        dV_virtual = LinearCtrlGain * tV;
        dV = dV_virtual/e_norm;
        dW_virtual = AngularCtrlGain * atan2(tW2,tW1);
        dW = (dW_virtual-dV*DksiCosSin)/dksi_dpsi;
        if isnan(dV) || isnan(dW)
            dV = 0.01;
            dW = 0.0;
        end
        

        [t,RobotPosition,RobotOrientation,RobotPositionTransformed,RobotOrientationTransformed,dV,dW,det(RobotPositionTransformedD)]

        dy = [dV*cos(RobotOrientation), dV*sin(RobotOrientation), dW]';
    end

    function status = output_function(t,y,flag)
        switch flag
            case 'init'
                pathTransformed = [pathTransformed ; [RobotPositionTransformed RobotOrientationTransformed]];
                pathGoalTransformed = [pathGoalTransformed ; GoalTransformed];
            case ''
                pathTransformed = [pathTransformed ; [RobotPositionTransformed RobotOrientationTransformed]];
                pathGoalTransformed = [pathGoalTransformed ; GoalTransformed];
            case 'done'
%                 assignin('base','pathTransformed',pathTransformed);
        end
        status = 0;
    end
end

































