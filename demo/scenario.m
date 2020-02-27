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


% Several simulation scenarios for a disk-shaped robot navigating in a
% convex polygon populated with obstacles.  
%
% Input:
%   ScenarioID  : Simulation Scenario ID
%
%
% Output:
%   Scenario    : Scenario Settings
%                   Scenario.Map : Map of an environment populated with obstacles
%                       Map.Boundary : Convex workspace boundary, represented as a simple polygon
%                       Map.KnownObstacle : Classes of known obstacles, a cellular list of simple polygons
%                           KnownObstacle.Points              - Points of the obstacle
%                           KnownObstacle.DilatedPoints       - Points of the obstacle after dilation by the robot radius
%                       Map.Obstacle : Workspace obstacles, a cellular list of simple polygons
%                           Obstacle.Type             - Type of obstacle (0 if unknown)
%                           Obstacle.StartingPoint    - Starting point of the obstacle if known
%                           Obstacle.Rotation         - Rotation of the obstacle if known
%                           Obstacle.Points           - Points of the obstacle
%                           Obstacle.DilatedPoints    - Points of the obstacle after dilation by the robot radius
%                       Map.DiffeoParams : Parameters of the diffeomorphism to be used
%                           DiffeoParams.epsilon        - Approximately the cutoff distance for the diffeo of each obstacle, in the R-function formulation
%                           DiffeoParams.varepsilon     - Parameter tuning the size of each polygonal collar (approximately epsilon is a good number)
%                           DiffeoParams.mu1            - The exponent used in the C_{\infty} beta switches (approximately epsilon is a good number)
%                           DiffeoParams.mu2            - The exponent used in the C_{\infty} gamma switches (0.05 is a good number)
%                           DiffeoParams.p              - Defines the L-p norm used for the R-functions (20 is a good number)
%                   Scenario.Robot : A disk-shaped robot model
%                       Robot.Radius           - Robot body radius
%                       Robot.SmallRadius      - Point robot radius used for visualization
%                       Robot.BodyAngle        - Angle samples for constructing robot body polygon
%                       Robot.WheelAngle       - Angle samples for constructing robot wheel polygon 
%                       Robot.CtrlGain         - Control gain
%                       Robot.AngularCtrlGain  - Angular Control gain
%                       Robot.LinearCtrlGain   - Linear Control gain
%                       Robot.AngularCmdMax    - Maximum angular command
%                       Robot.LinearCmdMax     - Maximum linear command
%                       Robot.RelTol           - Relative tolerance 
%                       Robot.Camera           - A 2D camera model for object recognition
%                           Camera.Range           - Measurement range
%                           Camera.LidarRange      - LIDAR range in model space
%                           Camera.Infinity        - No detection value
%                           Camera.MinAngle        - Minimum angle
%                           Camera.MaxAngle        - Maximum angle
%                           Camera.NumSample       - Number of samples
%                           Camera.Resolution      - Angular resolution
%                           Camera.Angle           - Sampling angles
%                   Scenario.Start                   : Start configuration/polygon
%                   Scenario.Goal                    : Goal configuration/polygon
%                   Scenario.Type                    : Type of the robot to simulate (0 for fully actuated, 1 for unicycle, 2 for polygon-diffeo fully actuated, 3 for polygon-diffeo unicycle)
%                   Scenario.InitiallyKnown          : List of initially known obstacles (have to be known - for example, walls)
%                   Scenario.RRT : Parameters related to RRT simulations
%                           RRT.Samples              : Number of samples
%                           RRT.Epsilon              : Sampling radius around each node (m)
%                           RRT.BallRadius           : Admissible radius to connect samples (m)
%                           RRT.PathDiscretization   : Discretization of the generated RRT path (m)
%                           RRT.UpdateRate           : RRT update frequency (Hz)
%                   Scenario.ODE                     : ODE solver settings
%                           ODE.tspan                   - Integration Time interval
%                           ODE.option                  - Integrator options

function Scenario = scenario(ScenarioID, numSampleObstacle, numSampleCamera)

    switch nargin
        case 0 
            ScenarioID = 0; % Scenario ID
            numSampleObstacle = 30; % Number of obstacle surface samples
            numSampleCamera = 100; % Number of LIDAR/Camera angular samples
        case 1
            numSampleObstacle = 30; % Number of obstacle surface samples
            numSampleCamera = 100; % Number of LIDAR/Camera angular samples 
        case 2
            numSampleCamera = 100; % Number of LIDAR/Camera angular samples 
    end

    switch ScenarioID
        case 1
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.2;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.2;
            Scenario.Robot.LinearCmdMax = 1.0;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2;
            Scenario.Map.DiffeoParams.varepsilon = 2;
            Scenario.Map.DiffeoParams.mu1 = 4; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.01; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; 1.5,-1.5,-1.5,1.5,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-3,-3,3,3,-3 ; 3,0,0,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{6}.Points = [2.518, 1.83, 2.043, 2.406, 2.655, 2.518 ; 0.5048, 0.2963, -0.2348, -0.8039, -0.0533, 0.5048];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{7}.Points = [4,8,8,4,4 ; 4,4,8,8,4];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 7;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
                        
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
            Scenario.Start = [1 6 0];
%             Scenario.Start = [2 5 0 ; 4 5 0 ; 2 8 0 ; 2 2 0 ; 5 9 0 ; 5 1 0];
            Scenario.Goal = [11, 6];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 610];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        case 2
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.2;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 1.0;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2;
            Scenario.Map.DiffeoParams.varepsilon = 2;
            Scenario.Map.DiffeoParams.mu1 = 4; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; 1.5,-1.5,-1.5,1.5,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-3,-3,3,3,-3 ; 3,0,0,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{6}.Points = [2.518, 1.83, 2.043, 2.406, 2.655, 2.518 ; 0.5048, 0.2963, -0.2348, -0.8039, -0.0533, 0.5048];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{7}.Points = [6,8,8,6,6,7,7,6,6 ; 4,4,8,8,7,7,5,5,4];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 7;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
                        
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
            Scenario.Start = [1 6 0];
%             Scenario.Start = [2 5 0 ; 4 5 0 ; 2 8 0 ; 2 2 0 ; 5 9 0 ; 5 1 0];
            Scenario.Goal = [11, 6];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 200];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        case 3
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 1.0;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 1.0;
            Scenario.Robot.LinearCmdMax = 1.0;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2;
            Scenario.Map.DiffeoParams.varepsilon = 2;
            Scenario.Map.DiffeoParams.mu1 = 4; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; 1.5,-1.5,-1.5,1.5,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-3,-3,3,3,-3 ; 3,0,0,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{6}.Points = [2.518, 1.83, 2.043, 2.406, 2.655, 2.518 ; 0.5048, 0.2963, -0.2348, -0.8039, -0.0533, 0.5048];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{7}.Points = [-1,1,1,-1,-1 ; -0.5,-0.5,0.5,0.5,-0.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 7;
            Scenario.Map.Obstacle{1}.StartingPoint = [6,7.2];
            Scenario.Map.Obstacle{1}.Rotation = -pi/4;
            
            Scenario.Map.Obstacle{2}.Type = 7;
            Scenario.Map.Obstacle{2}.StartingPoint = [6,4.8];
            Scenario.Map.Obstacle{2}.Rotation = pi/4;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
            Scenario.Start = [1 6 0];
%             Scenario.Start = [2 5 0 ; 4 5 0 ; 2 8 0 ; 2 2 0 ; 5 9 0 ; 5 1 0];
            Scenario.Goal = [11, 6];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 100];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
        case 4
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 0.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 10; 10 10; 10 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2.0;
            Scenario.Map.DiffeoParams.varepsilon = 2.0;
            Scenario.Map.DiffeoParams.mu1 = 4; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4,6,6,4,4 ; 6,6,10,10,6];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [6 9 9 6 6 8 8 6 6 ; 10 10 14 14 13 13 11 11 10];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-0.5,0.5,0.5,-0.5,-0.5 ; -1,-1,1,1,-1];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [-3,-4.2];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 4;
            Scenario.Map.Obstacle{2}.StartingPoint = [-3,-8.4];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [1 1 0; 1 3 0 ; 1 5 0 ; 1 7 0 ; 1 9 0];
            Scenario.Goal = [9,9];
            Scenario.Type = [2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        case 5
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-3 -3; -3 10; 10 10; 10 -3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; 1.5,-1.5,-1.5,1.5,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-0.5,0.5,0.5,-0.5,-0.5 ; -1,-1,1,1,-1];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 5;
            Scenario.Map.Obstacle{1}.StartingPoint = [0.9,-1];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 5;
            Scenario.Map.Obstacle{2}.StartingPoint = [2.5,-0.5];
            Scenario.Map.Obstacle{2}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{3}.Type = 5;
            Scenario.Map.Obstacle{3}.StartingPoint = [4.1,-1];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 5;
            Scenario.Map.Obstacle{4}.StartingPoint = [6.8,-0.5];
            Scenario.Map.Obstacle{4}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0.9,1.1];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 5;
            Scenario.Map.Obstacle{6}.StartingPoint = [2.35,5];
            Scenario.Map.Obstacle{6}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{7}.Type = 5;
            Scenario.Map.Obstacle{7}.StartingPoint = [-1,5];
            Scenario.Map.Obstacle{7}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{8}.Type = 5;
            Scenario.Map.Obstacle{8}.StartingPoint = [4.05,4.5];
            Scenario.Map.Obstacle{8}.Rotation = 0;
            
            Scenario.Map.Obstacle{9}.Type = 5;
            Scenario.Map.Obstacle{9}.StartingPoint = [2.5,0.7];
            Scenario.Map.Obstacle{9}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{10}.Type = 5;
            Scenario.Map.Obstacle{10}.StartingPoint = [-0.7,1.6];
            Scenario.Map.Obstacle{10}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{11}.Type = 5;
            Scenario.Map.Obstacle{11}.StartingPoint = [-1.9,3.9];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{12}.Type = 5;
            Scenario.Map.Obstacle{12}.StartingPoint = [1.2,7.2];
            Scenario.Map.Obstacle{12}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{13}.Type = 5;
            Scenario.Map.Obstacle{13}.StartingPoint = [0.7,5.5];
            Scenario.Map.Obstacle{13}.Rotation = 0;
            
            Scenario.Map.Obstacle{14}.Type = 5;
            Scenario.Map.Obstacle{14}.StartingPoint = [-1.8,9.3];
            Scenario.Map.Obstacle{14}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{15}.Type = 5;
            Scenario.Map.Obstacle{15}.StartingPoint = [8.5,-1];
            Scenario.Map.Obstacle{15}.Rotation = 0;
            
            Scenario.Map.Obstacle{16}.Type = 5;
            Scenario.Map.Obstacle{16}.StartingPoint = [7.5,3.5];
            Scenario.Map.Obstacle{16}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{17}.Type = 5;
            Scenario.Map.Obstacle{17}.StartingPoint = [7.0,5.2];
            Scenario.Map.Obstacle{17}.Rotation = 0;
            
            Scenario.Map.Obstacle{18}.Type = 5;
            Scenario.Map.Obstacle{18}.StartingPoint = [2.9,7.7];
            Scenario.Map.Obstacle{18}.Rotation = 0;
            
            Scenario.Map.Obstacle{19}.Type = 5;
            Scenario.Map.Obstacle{19}.StartingPoint = [7.5,8.75];
            Scenario.Map.Obstacle{19}.Rotation = 0;
            
            Scenario.Map.Obstacle{20}.Type = 5;
            Scenario.Map.Obstacle{20}.StartingPoint = [9.2,3.0];
            Scenario.Map.Obstacle{20}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [4 7.5 0];
%             Scenario.Start = [-2,-2,0 ; 2.5,-2,0 ; 6,-2,0 ; 2.5,3.5,0 ; 2,6,0 ; 1.0,9,0 ; 9.5,-2,0 ; -2,6.5,0];
            Scenario.Start = [-2,-2,0 ; 2.5,-2,0 ; 6,-2,0 ; 2.5,3.5,0 ; 2,6,0 ; 1.0,9,0 ; 9.5,-2,0 ; -2,6.5,0];
            Scenario.Goal = [9,9];
            Scenario.Type = [2 2 2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 1500];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
        case 6
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-3 -3; -3 10; 10 10; 10 -3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 1.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; 1.5,-1.5,-1.5,1.5,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-0.5,0.5,0.5,-0.5,-0.5 ; -1,-1,1,1,-1];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 5;
            Scenario.Map.Obstacle{1}.StartingPoint = [0.9,-1];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 5;
            Scenario.Map.Obstacle{2}.StartingPoint = [2.5,-0.5];
            Scenario.Map.Obstacle{2}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{3}.Type = 5;
            Scenario.Map.Obstacle{3}.StartingPoint = [4.1,-1];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 5;
            Scenario.Map.Obstacle{4}.StartingPoint = [6.8,-0.5];
            Scenario.Map.Obstacle{4}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0.9,1.1];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 5;
            Scenario.Map.Obstacle{6}.StartingPoint = [2.35,5];
            Scenario.Map.Obstacle{6}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{7}.Type = 5;
            Scenario.Map.Obstacle{7}.StartingPoint = [-1,5];
            Scenario.Map.Obstacle{7}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{8}.Type = 5;
            Scenario.Map.Obstacle{8}.StartingPoint = [4.05,4.5];
            Scenario.Map.Obstacle{8}.Rotation = 0;
            
            Scenario.Map.Obstacle{9}.Type = 5;
            Scenario.Map.Obstacle{9}.StartingPoint = [2.5,0.7];
            Scenario.Map.Obstacle{9}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{10}.Type = 5;
            Scenario.Map.Obstacle{10}.StartingPoint = [-0.7,1.6];
            Scenario.Map.Obstacle{10}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{11}.Type = 5;
            Scenario.Map.Obstacle{11}.StartingPoint = [-1.9,3.9];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{12}.Type = 5;
            Scenario.Map.Obstacle{12}.StartingPoint = [1.2,7.2];
            Scenario.Map.Obstacle{12}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{13}.Type = 5;
            Scenario.Map.Obstacle{13}.StartingPoint = [0.7,5.5];
            Scenario.Map.Obstacle{13}.Rotation = 0;
            
            Scenario.Map.Obstacle{14}.Type = 5;
            Scenario.Map.Obstacle{14}.StartingPoint = [-1.8,9.3];
            Scenario.Map.Obstacle{14}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{15}.Type = 5;
            Scenario.Map.Obstacle{15}.StartingPoint = [8.5,-1];
            Scenario.Map.Obstacle{15}.Rotation = 0;
            
            Scenario.Map.Obstacle{16}.Type = 5;
            Scenario.Map.Obstacle{16}.StartingPoint = [7.5,3.5];
            Scenario.Map.Obstacle{16}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{17}.Type = 5;
            Scenario.Map.Obstacle{17}.StartingPoint = [7.0,5.2];
            Scenario.Map.Obstacle{17}.Rotation = 0;
            
            Scenario.Map.Obstacle{18}.Type = 5;
            Scenario.Map.Obstacle{18}.StartingPoint = [2.9,7.7];
            Scenario.Map.Obstacle{18}.Rotation = 0;
            
            Scenario.Map.Obstacle{19}.Type = 5;
            Scenario.Map.Obstacle{19}.StartingPoint = [7.5,8.75];
            Scenario.Map.Obstacle{19}.Rotation = 0;
            
            Scenario.Map.Obstacle{20}.Type = 5;
            Scenario.Map.Obstacle{20}.StartingPoint = [9.2,3.0];
            Scenario.Map.Obstacle{20}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [4 7.5 0];
%             Scenario.Start = [-2,-2,0 ; 2.5,-2,0 ; 6,-2,0 ; 2.5,3.5,0 ; 2,6,0 ; 1.0,9,0 ; 9.5,-2,0 ; -2,6.5,0];
            Scenario.Start = [-2,-2,0 ; 2.5,-2,0 ; 6,-2,0 ; 2.5,3.5,0 ; 2,6,0 ; 1.0,9,0 ; 9.5,-2,0 ; -2,6.5,0];
            Scenario.Goal = [9,9];
            Scenario.Type = [2 2 2 2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 1500];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        
        case 7
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.8;
            Scenario.Map.DiffeoParams.varepsilon = 0.8;
            Scenario.Map.DiffeoParams.mu1 = 1.6; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 2.4*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 2*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 3*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 4*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [3,2];
            Scenario.Map.Obstacle{1}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{2}.Type = 3;
            Scenario.Map.Obstacle{2}.StartingPoint = [2.5,3.5];
            Scenario.Map.Obstacle{2}.Rotation = -pi/6;
            
            Scenario.Map.Obstacle{3}.Type = 1;
            Scenario.Map.Obstacle{3}.StartingPoint = [2.5,8.7];
            Scenario.Map.Obstacle{3}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{4}.Type = 5;
            Scenario.Map.Obstacle{4}.StartingPoint = [9.8,2];
            Scenario.Map.Obstacle{4}.Rotation = pi/4;
            
            Scenario.Map.Obstacle{5}.Type = 2;
            Scenario.Map.Obstacle{5}.StartingPoint = [9.,8.7];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 3;
            Scenario.Map.Obstacle{6}.StartingPoint = [7.7,6.9];
            Scenario.Map.Obstacle{6}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{7}.Type = 3;
            Scenario.Map.Obstacle{7}.StartingPoint = [7.7,8.7];
            Scenario.Map.Obstacle{7}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{8}.Type = 3;
            Scenario.Map.Obstacle{8}.StartingPoint = [7.7,10.5];
            Scenario.Map.Obstacle{8}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{9}.Type = 3;
            Scenario.Map.Obstacle{9}.StartingPoint = [10.3,6.9];
            Scenario.Map.Obstacle{9}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{10}.Type = 3;
            Scenario.Map.Obstacle{10}.StartingPoint = [10.3,8.7];
            Scenario.Map.Obstacle{10}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{11}.Type = 3;
            Scenario.Map.Obstacle{11}.StartingPoint = [10.3,10.5];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{12}.Type = 0;
            Scenario.Map.Obstacle{12}.Points = obstacle2D_ball([4,5.25],0.5,100);
            
            Scenario.Map.Obstacle{13}.Type = 0;
            Scenario.Map.Obstacle{13}.Points = obstacle2D_ball([6,3],0.5,100);
            
            Scenario.Map.Obstacle{14}.Type = 0;
            Scenario.Map.Obstacle{14}.Points = obstacle2D_ball([5.8,7.2],0.5,100);
            
            Scenario.Map.Obstacle{15}.Type = 0;
            Scenario.Map.Obstacle{15}.Points = obstacle2D_ball([8.5,4.2],1,100);
            
            Scenario.Map.Obstacle{16}.Type = 0;
            Scenario.Map.Obstacle{16}.Points = obstacle2D_ball([1.7,6],0.7,100);
            
            Scenario.Map.Obstacle{17}.Type = 0;
            Scenario.Map.Obstacle{17}.Points = obstacle2D_ball([6,5],0.3,100);
            
            Scenario.Map.Obstacle{18}.Type = 0;
            Scenario.Map.Obstacle{18}.Points = obstacle2D_ball([5.0,9.5],0.5,100);
            
            Scenario.Map.Obstacle{19}.Type = 0;
            Scenario.Map.Obstacle{19}.Points = obstacle2D_ball([7.5,1.3],0.5,100);
            
            Scenario.Map.Obstacle{20}.Type = 0;
            Scenario.Map.Obstacle{20}.Points = obstacle2D_ball([4,7],0.3,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 11 0 ; 6 11.7 0 ; 3.2 7.5 0 ; 1 3 0 ; 4.2 4 0 ; 10.3 7.8 0];
            Scenario.Start = [6 11.7 0 ; 1 3 0 ; 1 11 0 ; 3.2 7.5 0 ; 4.2 4 0 ; 10.3 7.8 0];
%             Scenario.Start = [9 11.7 0];
            Scenario.Goal = [11.2 1];
            Scenario.Type = [2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 1500];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
        case 8
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.8;
            Scenario.Map.DiffeoParams.varepsilon = 0.8;
            Scenario.Map.DiffeoParams.mu1 = 1.6; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 2.4*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 2*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 3*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 4*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [3,2];
            Scenario.Map.Obstacle{1}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{2}.Type = 3;
            Scenario.Map.Obstacle{2}.StartingPoint = [2.5,3.5];
            Scenario.Map.Obstacle{2}.Rotation = -pi/6;
            
            Scenario.Map.Obstacle{3}.Type = 1;
            Scenario.Map.Obstacle{3}.StartingPoint = [2.5,8.7];
            Scenario.Map.Obstacle{3}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{4}.Type = 5;
            Scenario.Map.Obstacle{4}.StartingPoint = [9.8,2];
            Scenario.Map.Obstacle{4}.Rotation = pi/4;
            
            Scenario.Map.Obstacle{5}.Type = 2;
            Scenario.Map.Obstacle{5}.StartingPoint = [9.,8.7];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 3;
            Scenario.Map.Obstacle{6}.StartingPoint = [7.7,6.9];
            Scenario.Map.Obstacle{6}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{7}.Type = 3;
            Scenario.Map.Obstacle{7}.StartingPoint = [7.7,8.7];
            Scenario.Map.Obstacle{7}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{8}.Type = 3;
            Scenario.Map.Obstacle{8}.StartingPoint = [7.7,10.5];
            Scenario.Map.Obstacle{8}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{9}.Type = 3;
            Scenario.Map.Obstacle{9}.StartingPoint = [10.3,6.9];
            Scenario.Map.Obstacle{9}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{10}.Type = 3;
            Scenario.Map.Obstacle{10}.StartingPoint = [10.3,8.7];
            Scenario.Map.Obstacle{10}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{11}.Type = 3;
            Scenario.Map.Obstacle{11}.StartingPoint = [10.3,10.5];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{12}.Type = 0;
            Scenario.Map.Obstacle{12}.Points = obstacle2D_ball([4,5.25],0.5,100);
            
            Scenario.Map.Obstacle{13}.Type = 0;
            Scenario.Map.Obstacle{13}.Points = obstacle2D_ball([6,3],0.5,100);
            
            Scenario.Map.Obstacle{14}.Type = 0;
            Scenario.Map.Obstacle{14}.Points = obstacle2D_ball([5.8,7.2],0.5,100);
            
            Scenario.Map.Obstacle{15}.Type = 0;
            Scenario.Map.Obstacle{15}.Points = obstacle2D_ball([8.5,4.2],1,100);
            
            Scenario.Map.Obstacle{16}.Type = 0;
            Scenario.Map.Obstacle{16}.Points = obstacle2D_ball([1.7,6],0.7,100);
            
            Scenario.Map.Obstacle{17}.Type = 0;
            Scenario.Map.Obstacle{17}.Points = obstacle2D_ball([6,5],0.3,100);
            
            Scenario.Map.Obstacle{18}.Type = 0;
            Scenario.Map.Obstacle{18}.Points = obstacle2D_ball([5.0,9.5],0.5,100);
            
            Scenario.Map.Obstacle{19}.Type = 0;
            Scenario.Map.Obstacle{19}.Points = obstacle2D_ball([7.5,1.3],0.5,100);
            
            Scenario.Map.Obstacle{20}.Type = 0;
            Scenario.Map.Obstacle{20}.Points = obstacle2D_ball([4,7],0.3,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 11 0 ; 6 11.7 0 ; 3.2 7.5 0 ; 1 3 0 ; 4.2 4 0 ; 10.3 7.8 0];
%             Scenario.Start = [6 11.7 0 ; 1 3 0 ; 1 11 0 ; 3.2 7.5 0 ; 4.2 4 0 ; 10.3 7.8 0];
            Scenario.Start = [6 11.7 0 ; 1 3 0 ; 1 11 0 ; 3.2 7.5 0 ; 4.2 4 0 ; 10.3 7.8 0];
            Scenario.Goal = [11.2 1];
            Scenario.Type = [2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 1500];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        case 9
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 10; 10 10; 10 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2.0;
            Scenario.Map.DiffeoParams.varepsilon = 2.0;
            Scenario.Map.DiffeoParams.mu1 = 4.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4,6,6,4,4 ; 6,6,10,10,6];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [6 9 9 6 6 8 8 6 6 ; 10 10 14 14 13 13 11 11 10];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-0.5,0.5,0.5,-0.5,-0.5 ; -1,-1,1,1,-1];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [-3,-4.2];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 4;
            Scenario.Map.Obstacle{2}.StartingPoint = [-3,-8.4];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [1 1 0; 1 3 0 ; 1 5 0 ; 1 7 0 ; 1 9 0];
            Scenario.Goal = [9,9];
            Scenario.Type = [2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        case 10
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 10; 10 10; 10 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 2.0;
            Scenario.Map.DiffeoParams.varepsilon = 2.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4,6,6,4,4 ; 6,6,10,10,6];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [6 9 9 6 6 8 8 6 6 ; 10 10 14 14 13 13 11 11 10];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-0.5,0.5,0.5,-0.5,-0.5 ; -1,-1,1,1,-1];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [-3,-4.2];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 4;
            Scenario.Map.Obstacle{2}.StartingPoint = [-3,-8.4];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [1 1 0; 1 3 0 ; 1 5 0 ; 1 7 0 ; 1 9 0];
            Scenario.Goal = [9,9];
            Scenario.Type = [3 3 3 3 3 3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        case 11
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.8;
            Scenario.Map.DiffeoParams.varepsilon = 0.8;
            Scenario.Map.DiffeoParams.mu1 = 1.6; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 2.4*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 2*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 3*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 4*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [2.5,8.7];
            Scenario.Map.Obstacle{1}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{2}.Type = 5;
            Scenario.Map.Obstacle{2}.StartingPoint = [9.8,2];
            Scenario.Map.Obstacle{2}.Rotation = pi/4;
            
            Scenario.Map.Obstacle{3}.Type = 2;
            Scenario.Map.Obstacle{3}.StartingPoint = [9.,8.7];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 3;
            Scenario.Map.Obstacle{4}.StartingPoint = [7.7,6.9];
            Scenario.Map.Obstacle{4}.Rotation = -pi/2;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{5}.Type = 0;
            Scenario.Map.Obstacle{5}.Points = [6.0079    2.2501
                                                6.0392    2.2511
                                                6.0480    2.2515
                                                6.0549    2.2521
                                                6.0862    2.2551
                                                6.0949    2.2560
                                                6.1018    2.2570
                                                6.1328    2.2619
                                                6.1414    2.2634
                                                6.1482    2.2649
                                                6.1789    2.2717
                                                6.1874    2.2738
                                                6.1941    2.2756
                                                6.2243    2.2844
                                                6.2326    2.2870
                                                6.2392    2.2893
                                                6.2687    2.2999
                                                6.2769    2.3030
                                                6.2833    2.3057
                                                6.3122    2.3181
                                                6.3201    2.3218
                                                6.3264    2.3248
                                                6.3544    2.3391
                                                6.3621    2.3432
                                                6.3681    2.3467
                                                6.3952    2.3627
                                                6.4026    2.3672
                                                6.4084    2.3711
                                                6.4344    2.3887
                                                6.4415    2.3938
                                                6.4471    2.3980
                                                6.4719    2.4172
                                                6.4787    2.4227
                                                6.4840    2.4272
                                                6.5076    2.4480
                                                6.5140    2.4539
                                                6.5191    2.4587
                                                6.5413    2.4809
                                                6.5473    2.4872
                                                6.5520    2.4924
                                                6.5728    2.5160
                                                6.5784    2.5226
                                                6.5828    2.5281
                                                6.6020    2.5529
                                                6.6073    2.5599
                                                6.6113    2.5656
                                                6.6289    2.5916
                                                6.6337    2.5989
                                                6.6373    2.6048
                                                6.6533    2.6319
                                                6.6576    2.6395
                                                6.6609    2.6456
                                                6.6752    2.6736
                                                6.6790    2.6815
                                                6.6819    2.6878
                                                6.6943    2.7167
                                                6.6977    2.7247
                                                6.7001    2.7313
                                                6.7107    2.7608
                                                6.7136    2.7691
                                                6.7156    2.7757
                                                6.7244    2.8059
                                                6.7267    2.8143
                                                6.7283    2.8211
                                                6.7351    2.8518
                                                6.7369    2.8603
                                                6.7381    2.8672
                                                6.7430    2.8982
                                                6.7442    2.9069
                                                6.7449    2.9138
                                                6.7479    2.9451
                                                6.7486    2.9538
                                                6.7489    2.9608
                                                6.7499    2.9921
                                                6.7500    3.0009
                                                6.7499    3.0079
                                                6.7489    3.0392
                                                6.7485    3.0480
                                                6.7479    3.0549
                                                6.7449    3.0862
                                                6.7440    3.0949
                                                6.7430    3.1018
                                                6.7381    3.1328
                                                6.7366    3.1414
                                                6.7351    3.1482
                                                6.7283    3.1789
                                                6.7262    3.1874
                                                6.7244    3.1941
                                                6.7156    3.2243
                                                6.7130    3.2326
                                                6.7107    3.2392
                                                6.7001    3.2687
                                                6.6970    3.2769
                                                6.6943    3.2833
                                                6.6819    3.3122
                                                6.6782    3.3201
                                                6.6752    3.3264
                                                6.6609    3.3544
                                                6.6568    3.3621
                                                6.6533    3.3681
                                                6.6373    3.3952
                                                6.6328    3.4026
                                                6.6289    3.4084
                                                6.6113    3.4344
                                                6.6062    3.4415
                                                6.6020    3.4471
                                                6.5828    3.4719
                                                6.5773    3.4787
                                                6.5728    3.4840
                                                6.5520    3.5076
                                                6.5461    3.5140
                                                6.5413    3.5191
                                                6.5191    3.5413
                                                6.5128    3.5473
                                                6.5076    3.5520
                                                6.4840    3.5728
                                                6.4774    3.5784
                                                6.4719    3.5828
                                                6.4471    3.6020
                                                6.4401    3.6073
                                                6.4344    3.6113
                                                6.4084    3.6289
                                                6.4011    3.6337
                                                6.3952    3.6373
                                                6.3681    3.6533
                                                6.3605    3.6576
                                                6.3544    3.6609
                                                6.3264    3.6752
                                                6.3185    3.6790
                                                6.3122    3.6819
                                                6.2833    3.6943
                                                6.2753    3.6977
                                                6.2687    3.7001
                                                6.2392    3.7107
                                                6.2309    3.7136
                                                6.2243    3.7156
                                                6.1941    3.7244
                                                6.1857    3.7267
                                                6.1789    3.7283
                                                6.1482    3.7351
                                                6.1397    3.7369
                                                6.1328    3.7381
                                                6.1018    3.7430
                                                6.0931    3.7442
                                                6.0862    3.7449
                                                6.0773    3.7458
                                                6.0686    3.7465
                                                6.0599    3.7468
                                                6.0512    3.7469
                                                6.0424    3.7466
                                                6.0337    3.7461
                                                6.0250    3.7452
                                                6.0164    3.7441
                                                6.0078    3.7426
                                                5.9992    3.7409
                                                5.9908    3.7388
                                                5.9824    3.7365
                                                5.9740    3.7338
                                                5.9658    3.7309
                                                5.9577    3.7277
                                                5.9497    3.7242
                                                5.9418    3.7204
                                                5.9341    3.7164
                                                5.9265    3.7121
                                                5.9191    3.7075
                                                5.9118    3.7027
                                                5.9047    3.6976
                                                5.8978    3.6923
                                                5.8911    3.6867
                                                5.8846    3.6809
                                                5.8782    3.6749
                                                5.8721    3.6687
                                                5.8663    3.6622
                                                5.8606    3.6556
                                                5.8552    3.6487
                                                5.8500    3.6417
                                                5.8451    3.6345
                                                5.8404    3.6271
                                                5.8360    3.6196
                                                5.8318    3.6119
                                                5.8279    3.6041
                                                5.8243    3.5962
                                                5.8210    3.5881
                                                5.8168    3.5764
                                                5.8119    3.5619
                                                5.8106    3.5578
                                                5.8000    3.5244
                                                5.7987    3.5202
                                                5.7887    3.4867
                                                5.7875    3.4825
                                                5.7782    3.4487
                                                5.7770    3.4445
                                                5.7682    3.4106
                                                5.7672    3.4064
                                                5.7590    3.3724
                                                5.7580    3.3681
                                                5.7504    3.3339
                                                5.7495    3.3297
                                                5.7425    3.2954
                                                5.7417    3.2911
                                                5.7353    3.2566
                                                5.7345    3.2523
                                                5.7287    3.2178
                                                5.7280    3.2135
                                                5.7228    3.1789
                                                5.7222    3.1745
                                                5.7176    3.1398
                                                5.7171    3.1355
                                                5.7131    3.1007
                                                5.7127    3.0964
                                                5.7093    3.0615
                                                5.7089    3.0572
                                                5.7062    3.0223
                                                5.7059    3.0179
                                                5.7037    2.9830
                                                5.7035    2.9786
                                                5.7020    2.9436
                                                5.7018    2.9393
                                                5.7009    2.9043
                                                5.7008    2.8999
                                                5.7005    2.8649
                                                5.7005    2.8605
                                                5.7008    2.8255
                                                5.7009    2.8211
                                                5.7018    2.7862
                                                5.7020    2.7818
                                                5.7035    2.7468
                                                5.7037    2.7424
                                                5.7059    2.7075
                                                5.7062    2.7031
                                                5.7089    2.6682
                                                5.7093    2.6639
                                                5.7127    2.6290
                                                5.7131    2.6247
                                                5.7171    2.5899
                                                5.7176    2.5856
                                                5.7222    2.5509
                                                5.7228    2.5465
                                                5.7280    2.5119
                                                5.7287    2.5076
                                                5.7345    2.4731
                                                5.7353    2.4688
                                                5.7378    2.4548
                                                5.7396    2.4463
                                                5.7416    2.4378
                                                5.7440    2.4294
                                                5.7466    2.4211
                                                5.7495    2.4128
                                                5.7527    2.4047
                                                5.7562    2.3967
                                                5.7600    2.3889
                                                5.7640    2.3811
                                                5.7683    2.3735
                                                5.7729    2.3661
                                                5.7777    2.3588
                                                5.7827    2.3517
                                                5.7881    2.3448
                                                5.7936    2.3381
                                                5.7994    2.3315
                                                5.8054    2.3252
                                                5.8116    2.3191
                                                5.8181    2.3132
                                                5.8247    2.3075
                                                5.8315    2.3021
                                                5.8386    2.2969
                                                5.8458    2.2920
                                                5.8531    2.2873
                                                5.8606    2.2829
                                                5.8683    2.2787
                                                5.8761    2.2748
                                                5.8841    2.2712
                                                5.8921    2.2679
                                                5.9003    2.2648
                                                5.9086    2.2620
                                                5.9169    2.2596
                                                5.9254    2.2574
                                                5.9339    2.2555
                                                5.9425    2.2539
                                                5.9511    2.2526
                                                5.9598    2.2517
                                                5.9685    2.2510
                                                5.9758    2.2506
                                                5.9921    2.2501
                                                6.0009    2.2500
                                                6.0079    2.2501];
            
            Scenario.Map.Obstacle{6}.Type = 0;
            Scenario.Map.Obstacle{6}.Points = [8.5079    2.9501
                                                8.5706    2.9521
                                                8.5794    2.9525
                                                8.5863    2.9531
                                                8.6489    2.9590
                                                8.6575    2.9600
                                                8.6644    2.9610
                                                8.7265    2.9708
                                                8.7351    2.9723
                                                8.7419    2.9737
                                                8.8032    2.9874
                                                8.8117    2.9895
                                                8.8184    2.9913
                                                8.8788    3.0089
                                                8.8871    3.0115
                                                8.8937    3.0137
                                                8.9528    3.0350
                                                8.9610    3.0381
                                                8.9674    3.0408
                                                9.0251    3.0657
                                                9.0330    3.0693
                                                9.0393    3.0724
                                                9.0953    3.1009
                                                9.1030    3.1050
                                                9.1090    3.108514
                                                9.1631    3.1405
                                                9.1705    3.1451
                                                9.1763    3.1489
                                                9.2283    3.1842
                                                9.2354    3.1892
                                                9.2410    3.1934
                                                9.2907    3.2319
                                                9.2975    3.2374
                                                9.3028    3.2420
                                                9.3499    3.2835
                                                9.3563    3.2894
                                                9.3613    3.2943
                                                9.4057    3.3387
                                                9.4118    3.3450
                                                9.4165    3.3501
                                                9.4580    3.3972
                                                9.4637    3.4039
                                                9.4681    3.4093
                                                9.5066    3.4590
                                                9.5118    3.4660
                                                9.5158    3.4717
                                                9.5511    3.5237
                                                9.5559    3.5310
                                                9.5595    3.5369
                                                9.5915    3.5910
                                                9.5958    3.5986
                                                9.5991    3.6047
                                                9.6276    3.6607
                                                9.6314    3.6686
                                                9.6343    3.6749
                                                9.6592    3.7326
                                                9.6625    3.7407
                                                9.6650    3.7472
                                                9.6863    3.8063
                                                9.6891    3.8146
                                                9.6911    3.8212
                                                9.7087    3.8816
                                                9.7109    3.8900
                                                9.7126    3.8968
                                                9.7263    3.9581
                                                9.7280    3.9666
                                                9.7292    3.9735
                                                9.7390    4.0356
                                                9.7403    4.0442
                                                9.7410    4.0511
                                                9.7463    4.1068
                                                9.7469    4.1155
                                                9.7473    4.1242
                                                9.7474    4.1330
                                                9.7471    4.1417
                                                9.7466    4.1504
                                                9.7457    4.1591
                                                9.7446    4.1677
                                                9.7431    4.1763
                                                9.7414    4.1849
                                                9.7393    4.1934
                                                9.7370    4.2018
                                                9.7343    4.2101
                                                9.7314    4.2183
                                                9.7282    4.2264
                                                9.7247    4.2344
                                                9.7209    4.2423
                                                9.7169    4.2500
                                                9.7126    4.2576
                                                9.7080    4.2650
                                                9.7032    4.2723
                                                9.6981    4.2794
                                                9.6914    4.2880
                                                9.6853    4.2955
                                                9.6825    4.2989
                                                9.6599    4.3257
                                                9.6571    4.3290
                                                9.6341    4.3554
                                                9.6312    4.3586
                                                9.6077    4.3846
                                                9.6047    4.3878
                                                9.5808    4.4134
                                                9.5778    4.4165
                                                9.5534    4.4417
                                                9.5503    4.4448
                                                9.5255    4.4695
                                                9.5224    4.4725
                                                9.4972    4.4968
                                                9.4940    4.4998
                                                9.4683    4.5236
                                                9.4651    4.5266
                                                9.4390    4.5499
                                                9.4358    4.5528
                                                9.4093    4.5757
                                                9.4060    4.5785
                                                9.3791    4.6010
                                                9.3757    4.6038
                                                9.3484    4.6257
                                                9.3450    4.6284
                                                9.3174    4.6499
                                                9.3139    4.6526
                                                9.2859    4.6736
                                                9.2824    4.6761
                                                9.2540    4.6966
                                                9.2504    4.6992
                                                9.2217    4.7192
                                                9.2181    4.7217
                                                9.1890    4.7411
                                                9.1854    4.7435
                                                9.1559    4.7625
                                                9.1523    4.7649
                                                9.1225    4.7833
                                                9.1188    4.7856
                                                9.0887    4.8036
                                                9.0849    4.8058
                                                9.0546    4.8232
                                                9.0508    4.8253
                                                9.0201    4.8422
                                                9.0163    4.8443
                                                8.9853    4.8606
                                                8.9814    4.8626
                                                8.9502    4.8784
                                                8.9462    4.8804
                                                8.9147    4.8956
                                                8.9108    4.8975
                                                8.8790    4.9122
                                                8.8750    4.9140
                                                8.8430    4.9281
                                                8.8390    4.9298
                                                8.8067    4.9434
                                                8.8027    4.9451
                                                8.7702    4.9581
                                                8.7661    4.9597
                                                8.7334    4.9721
                                                8.7293    4.9736
                                                8.6963    4.9855
                                                8.6922    4.9869
                                                8.6591    4.9982
                                                8.6549    4.9996
                                                8.6216    5.0103
                                                8.6174    5.0116
                                                8.5839    5.0217
                                                8.5797    5.0229
                                                8.5460    5.0325
                                                8.5418    5.0336
                                                8.5080    5.0425
                                                8.5037    5.0436
                                                8.4697    5.0520
                                                8.4655    5.0530
                                                8.4313    5.0607
                                                8.4271    5.0616
                                                8.3928    5.0688
                                                8.3885    5.0697
                                                8.3541    5.0762
                                                8.3498    5.0770
                                                8.3153    5.0829
                                                8.3110    5.0836
                                                8.2764    5.0890
                                                8.2721    5.0896
                                                8.2374    5.0943
                                                8.2331    5.0949
                                                8.1983    5.0990
                                                8.1939    5.0995
                                                8.1591    5.1030
                                                8.1548    5.1034
                                                8.1199    5.1063
                                                8.1155    5.1066
                                                8.0806    5.1089
                                                8.0762    5.1092
                                                8.0413    5.1109
                                                8.0369    5.1110
                                                8.0019    5.1121
                                                7.9975    5.1122
                                                7.9625    5.1127
                                                7.9581    5.1127
                                                7.9231    5.1125
                                                7.9188    5.1125
                                                7.8838    5.1117
                                                7.8794    5.1116
                                                7.8444    5.1102
                                                7.8401    5.1100
                                                7.8051    5.1080
                                                7.8007    5.1077
                                                7.7658    5.1051
                                                7.7615    5.1047
                                                7.7266    5.1015
                                                7.7223    5.1011
                                                7.7148    5.1003
                                                7.7062    5.0992
                                                7.6976    5.0978
                                                7.6890    5.0961
                                                7.6805    5.0941
                                                7.6721    5.0918
                                                7.6638    5.0892
                                                7.6555    5.0863
                                                7.6474    5.0831
                                                7.6394    5.0797
                                                7.6315    5.0760
                                                7.6237    5.0720
                                                7.6161    5.0677
                                                7.6087    5.0632
                                                7.6014    5.0584
                                                7.5942    5.0533
                                                7.5873    5.0481
                                                7.5805    5.0425
                                                7.5740    5.0368
                                                7.5676    5.0308
                                                7.5615    5.0246
                                                7.5546    5.0171
                                                7.5420    5.0028
                                                7.5363    4.9961
                                                7.5319    4.9907
                                                7.4934    4.9410
                                                7.4882    4.9340
                                                7.4842    4.9283
                                                7.4489    4.8763
                                                7.4441    4.8690
                                                7.4405    4.8631
                                                7.4085    4.8090
                                                7.4042    4.8014
                                                7.4009    4.7953
                                                7.3724    4.7393
                                                7.3686    4.7314
                                                7.3657    4.7251
                                                7.3408    4.6674
                                                7.3375    4.6593
                                                7.3350    4.6528
                                                7.3137    4.5937
                                                7.3109    4.5854
                                                7.3089    4.5788
                                                7.2913    4.5184
                                                7.2891    4.5100
                                                7.2874    4.5032
                                                7.2737    4.4419
                                                7.2720    4.4334
                                                7.2708    4.4265
                                                7.2610    4.3644
                                                7.2597    4.3558
                                                7.2590    4.3489
                                                7.2531    4.2863
                                                7.2524    4.2776
                                                7.2521    4.2706
                                                7.2501    4.2079
                                                7.2500    4.1991
                                                7.2501    4.1921
                                                7.2521    4.1294
                                                7.2525    4.1206
                                                7.2531    4.1137
                                                7.2590    4.0511
                                                7.2600    4.0425
                                                7.2610    4.0356
                                                7.2708    3.9735
                                                7.2723    3.9649
                                                7.2737    3.9581
                                                7.2874    3.8968
                                                7.2895    3.8883
                                                7.2913    3.8816
                                                7.3089    3.8212
                                                7.3115    3.8129
                                                7.3137    3.8063
                                                7.3350    3.7472
                                                7.3381    3.7390
                                                7.3408    3.7326
                                                7.3657    3.6749
                                                7.3693    3.6670
                                                7.3724    3.6607
                                                7.4009    3.6047
                                                7.4050    3.5970
                                                7.4085    3.5910
                                                7.4405    3.5369
                                                7.4451    3.5295
                                                7.4489    3.5237
                                                7.4842    3.4717
                                                7.4892    3.4646
                                                7.4934    3.4590
                                                7.5319    3.4093
                                                7.5374    3.4025
                                                7.5420    3.3972
                                                7.5835    3.3501
                                                7.5894    3.3437
                                                7.5943    3.3387
                                                7.6387    3.2943
                                                7.6450    3.2882
                                                7.6501    3.2835
                                                7.6972    3.2420
                                                7.7039    3.2363
                                                7.7093    3.2319
                                                7.7590    3.1934
                                                7.7660    3.1882
                                                7.7717    3.1842
                                                7.8237    3.1489
                                                7.8310    3.1441
                                                7.8369    3.1405
                                                7.8910    3.1085
                                                7.8986    3.1042
                                                7.9047    3.1009
                                                7.9607    3.0724
                                                7.9686    3.0686
                                                7.9749    3.0657
                                                8.0326    3.0408
                                                8.0407    3.0375
                                                8.0472    3.0350
                                                8.1063    3.0137
                                                8.1146    3.0109
                                                8.1212    3.0089
                                                8.1816    2.9913
                                                8.1900    2.9891
                                                8.1968    2.9874
                                                8.2581    2.9737
                                                8.2666    2.9720
                                                8.2735    2.9708
                                                8.3356    2.9610
                                                8.3442    2.9597
                                                8.3511    2.9590
                                                8.4137    2.9531
                                                8.4224    2.9524
                                                8.4294    2.9521
                                                8.4921    2.9501
                                                8.5009    2.9500
                                                8.5079    2.9501];
            
            Scenario.Map.Obstacle{7}.Type = 0;
            Scenario.Map.Obstacle{7}.Points = [7.7700    0.6223
                                                7.7787    0.6229
                                                7.7874    0.6238
                                                7.7961    0.6249
                                                7.8047    0.6264
                                                7.8132    0.6282
                                                7.8217    0.6303
                                                7.8301    0.6326
                                                7.8384    0.6353
                                                7.8466    0.6382
                                                7.8547    0.6415
                                                7.8627    0.6450
                                                7.8706    0.6488
                                                7.8783    0.6528
                                                7.8854    0.6569
                                                7.8952    0.6627
                                                7.9026    0.6672
                                                7.9084    0.6711
                                                7.9344    0.6887
                                                7.9415    0.6938
                                                7.9471    0.6980
                                                7.9719    0.7172
                                                7.9787    0.7227
                                                7.9840    0.7272
                                                8.0076    0.7480
                                                8.0140    0.7539
                                                8.0191    0.7587
                                                8.0413    0.7809
                                                8.0473    0.7872
                                                8.0520    0.7924
                                                8.0728    0.8160
                                                8.0784    0.8226
                                                8.0828    0.8281
                                                8.1020    0.8529
                                                8.1073    0.8599
                                                8.1113    0.8656
                                                8.1289    0.8916
                                                8.1337    0.8989
                                                8.1373    0.9048
                                                8.1533    0.9319
                                                8.1576    0.9395
                                                8.1609    0.9456
                                                8.1752    0.9736
                                                8.1790    0.981514
                                                8.1819    0.9878
                                                8.1943    1.0167
                                                8.1977    1.0247
                                                8.2001    1.0313
                                                8.2107    1.0608
                                                8.2136    1.0691
                                                8.2156    1.0757
                                                8.2244    1.1059
                                                8.2267    1.1143
                                                8.2283    1.1211
                                                8.2351    1.1518
                                                8.2369    1.1603
                                                8.2381    1.1672
                                                8.2430    1.1982
                                                8.2442    1.2069
                                                8.2449    1.2138
                                                8.2479    1.2451
                                                8.2486    1.2538
                                                8.2489    1.2608
                                                8.2499    1.2921
                                                8.2500    1.3009
                                                8.2499    1.3079
                                                8.2489    1.3392
                                                8.2485    1.3480
                                                8.2479    1.3549
                                                8.2449    1.3862
                                                8.2440    1.3949
                                                8.2430    1.4018
                                                8.2381    1.4328
                                                8.2366    1.4414
                                                8.2351    1.4482
                                                8.2283    1.4789
                                                8.2262    1.4874
                                                8.2244    1.4941
                                                8.2156    1.5243
                                                8.2130    1.5326
                                                8.2107    1.5392
                                                8.2001    1.5687
                                                8.1970    1.5769
                                                8.1943    1.5833
                                                8.1819    1.6122
                                                8.1782    1.6201
                                                8.1752    1.6264
                                                8.1609    1.6544
                                                8.1568    1.6621
                                                8.1533    1.6681
                                                8.1373    1.6952
                                                8.1328    1.7026
                                                8.1289    1.7084
                                                8.1113    1.7344
                                                8.1062    1.7415
                                                8.1020    1.7471
                                                8.0828    1.7719
                                                8.0773    1.7787
                                                8.0728    1.7840
                                                8.0520    1.8076
                                                8.0461    1.8140
                                                8.0413    1.8191
                                                8.0191    1.8413
                                                8.0128    1.8473
                                                8.0076    1.8520
                                                7.9840    1.8728
                                                7.9774    1.8784
                                                7.9719    1.8828
                                                7.9471    1.9020
                                                7.9401    1.9073
                                                7.9344    1.9113
                                                7.9084    1.9289
                                                7.9011    1.9337
                                                7.8952    1.9373
                                                7.8681    1.9533
                                                7.8605    1.9576
                                                7.8544    1.9609
                                                7.8264    1.9752
                                                7.8185    1.9790
                                                7.8122    1.9819
                                                7.7833    1.9943
                                                7.7753    1.9977
                                                7.7687    2.0001
                                                7.7392    2.0107
                                                7.7309    2.0136
                                                7.7243    2.0156
                                                7.6941    2.0244
                                                7.6857    2.0267
                                                7.6789    2.0283
                                                7.6482    2.0351
                                                7.6397    2.0369
                                                7.6328    2.0381
                                                7.6018    2.0430
                                                7.5931    2.0442
                                                7.5862    2.0449
                                                7.5549    2.0479
                                                7.5462    2.0486
                                                7.5392    2.0489
                                                7.5079    2.0499
                                                7.4991    2.0500
                                                7.4921    2.0499
                                                7.4608    2.0489
                                                7.4520    2.0485
                                                7.4451    2.0479
                                                7.4138    2.0449
                                                7.4051    2.0440
                                                7.3982    2.0430
                                                7.3672    2.0381
                                                7.3586    2.0366
                                                7.3518    2.0351
                                                7.3211    2.0283
                                                7.3126    2.0262
                                                7.3059    2.0244
                                                7.2757    2.0156
                                                7.2674    2.0130
                                                7.2608    2.0107
                                                7.2313    2.0001
                                                7.2231    1.9970
                                                7.2167    1.9943
                                                7.1878    1.9819
                                                7.1799    1.9782
                                                7.1736    1.9752
                                                7.1456    1.9609
                                                7.1379    1.9568
                                                7.1319    1.9533
                                                7.1048    1.9373
                                                7.0974    1.9328
                                                7.0916    1.9289
                                                7.0656    1.9113
                                                7.0585    1.9062
                                                7.0529    1.9020
                                                7.0281    1.8828
                                                7.0213    1.8773
                                                7.0160    1.8728
                                                6.9924    1.8520
                                                6.9860    1.8461
                                                6.9809    1.8413
                                                6.9587    1.8191
                                                6.9527    1.8128
                                                6.9480    1.8076
                                                6.9272    1.7840
                                                6.9216    1.7774
                                                6.9172    1.7719
                                                6.8980    1.7471
                                                6.8927    1.7401
                                                6.8887    1.7344
                                                6.8711    1.7084
                                                6.8663    1.7011
                                                6.8627    1.6952
                                                6.8467    1.6681
                                                6.8424    1.6605
                                                6.8391    1.6544
                                                6.8248    1.6264
                                                6.8210    1.6185
                                                6.8181    1.6122
                                                6.8057    1.5833
                                                6.8023    1.5753
                                                6.7999    1.5687
                                                6.7893    1.5392
                                                6.7864    1.5309
                                                6.7844    1.5243
                                                6.7756    1.4941
                                                6.7733    1.4857
                                                6.7717    1.4789
                                                6.7649    1.4482
                                                6.7631    1.4397
                                                6.7619    1.4328
                                                6.7570    1.4018
                                                6.7558    1.3931
                                                6.7551    1.3862
                                                6.7521    1.3549
                                                6.7514    1.3462
                                                6.7511    1.3392
                                                6.7501    1.3079
                                                6.7500    1.2991
                                                6.7501    1.2921
                                                6.7511    1.2608
                                                6.7515    1.2520
                                                6.7521    1.2451
                                                6.7551    1.2138
                                                6.7560    1.2051
                                                6.7570    1.1982
                                                6.7619    1.1672
                                                6.7634    1.1586
                                                6.7649    1.1518
                                                6.7717    1.1211
                                                6.7738    1.1126
                                                6.7756    1.1059
                                                6.7844    1.0757
                                                6.7870    1.0674
                                                6.7893    1.0608
                                                6.7999    1.0313
                                                6.8030    1.0231
                                                6.8057    1.0167
                                                6.8181    0.9878
                                                6.8218    0.9799
                                                6.8248    0.9736
                                                6.8310    0.9616
                                                6.8351    0.9539
                                                6.8394    0.9463
                                                6.8440    0.9389
                                                6.8489    0.9317
                                                6.8541    0.9246
                                                6.8594    0.9178
                                                6.8650    0.9111
                                                6.8709    0.9046
                                                6.8769    0.8983
                                                6.8832    0.8922
                                                6.8897    0.8864
                                                6.8964    0.8808
                                                6.9033    0.8754
                                                6.9103    0.8703
                                                6.9176    0.8654
                                                6.9250    0.8608
                                                6.9325    0.8564
                                                6.9429    0.8510
                                                6.9685    0.8383
                                                6.9724    0.8364
                                                7.0041    0.8214
                                                7.0080    0.8196
                                                7.0399    0.8052
                                                7.0439    0.8034
                                                7.0761    0.7896
                                                7.0801    0.7879
                                                7.1125    0.7746
                                                7.1166    0.7730
                                                7.1492    0.7602
                                                7.1533    0.7587
                                                7.1861    0.7465
                                                7.1902    0.7451
                                                7.2232    0.7335
                                                7.2274    0.7321
                                                7.2606    0.7211
                                                7.2648    0.7198
                                                7.2982    0.7093
                                                7.3024    0.7081
                                                7.3360    0.6983
                                                7.3402    0.6971
                                                7.3740    0.6878
                                                7.3782    0.6867
                                                7.4121    0.6781
                                                7.4164    0.6770
                                                7.4504    0.6690
                                                7.4547    0.6680
                                                7.4889    0.6606
                                                7.4932    0.6597
                                                7.5275    0.6528
                                                7.5318    0.6520
                                                7.5662    0.6458
                                                7.5706    0.6450
                                                7.6051    0.6394
                                                7.6094    0.6387
                                                7.6441    0.6337
                                                7.6484    0.6331
                                                7.6831    0.6287
                                                7.6875    0.6281
                                                7.7223    0.6243
                                                7.7266    0.6239
                                                7.7352    0.6231
                                                7.7439    0.6224
                                                7.7526    0.6221
                                                7.7613    0.6221
                                                7.7700    0.6223];
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 11 0 ; 6 11.7 0 ; 3.2 7.5 0 ; 1 3 0 ; 4.2 4 0 ; 10.3 7.8 0];
%             Scenario.Start = [6 11.7 0 ; 1 3 0 ; 1 11 0 ; 3.2 7.5 0 ; 4.2 4 0 ; 10.3 7.8 0];
            Scenario.Start = [1 11 0];
            Scenario.Goal = [11.2 1];
            Scenario.Type = [2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 1500];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
            
            
            
            
            
        % TABLECLOSED VISION OFF    
        case 12
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.7344   -0.3809 ; 4.5895    0.7558 ; 4.0650    0.7214 ; 4.2108   -0.4760 ; 4.7344   -0.3809]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TABLECLOSED VISION ON    
        case 13
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.7344   -0.3809 ; 4.5895    0.7558 ; 4.0650    0.7214 ; 4.2108   -0.4760 ; 4.7344   -0.3809]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
          
        % TWO CHAIRS VISION OFF
        case 14
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.0343   0.1730 ; 4.1048    0.1513 ; 4.2778    0.6393 ; 3.8797   0.8181 ; 3.6502   0.3531 ; 4.0343  0.1730]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.3980   -0.7275 ; 4.2124    -0.2430 ; 3.7600    -0.3824 ; 3.9976   -0.8985 ; 4.3980   -0.7275]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TWO CHAIRS VISION ON
        case 15
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.0343   0.1730 ; 4.1048    0.1513 ; 4.2778    0.6393 ; 3.8797   0.8181 ; 3.6502   0.3531 ; 4.0343  0.1730]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.3980   -0.7275 ; 4.2124    -0.2430 ; 3.7600    -0.3824 ; 3.9976   -0.8985 ; 4.3980   -0.7275]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TWO CHAIRS APART    
        case 16
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.2358    1.3353 ; 3.8828    1.5913 ; 3.5653    1.1829 ; 3.9690    0.8929 ; 4.2358    1.3353]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.4608   -0.7666 ; 4.2372   -0.3802 ; 4.0678   -0.3935 ; 3.7099   -0.7169 ; 4.0619   -1.0834 ; 4.4608   -0.7666]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TWO CHAIRS AND TABLE
        case 17
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.2358    1.3353 ; 3.8828    1.5913 ; 3.5653    1.1829 ; 3.9690    0.8929 ; 4.2358    1.3353]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.4608   -0.7666 ; 4.2372   -0.3802 ; 4.0678   -0.3935 ; 3.7099   -0.7169 ; 4.0619   -1.0834 ; 4.4608   -0.7666]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.5693   -0.2155 ; 4.3622    0.9116 ; 3.9057    0.8611 ; 3.8443    0.7728 ; 4.0373   -0.2735 ; 4.1151   -0.3268 ; 4.5693   -0.2155]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TWO GASCANS VISION OFF    
        case 18
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.8499    0.4150 ; 3.6604    0.6241 ; 3.5120    0.4859 ; 3.6795    0.3161 ; 3.9429    0.3092 ; 3.8499    0.4150]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.5133   -0.3376 ; 3.6940   -0.1772 ; 3.7600    0.0084 ; 3.3984   -0.2070 ; 3.5133   -0.3376]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % TWO GASCANS VISION ON    
        case 19
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.8499    0.4150 ; 3.6604    0.6241 ; 3.5120    0.4859 ; 3.6795    0.3161 ; 3.9429    0.3092 ; 3.8499    0.4150]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.5133   -0.3376 ; 3.6940   -0.1772 ; 3.7600    0.0084 ; 3.3984   -0.2070 ; 3.5133   -0.3376]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % THREE CHAIRS APART
        case 20
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [5.5422   -2.6681 ; 5.5614   -2.2221 ; 5.0448   -2.1884 ; 5.0308   -2.6847 ; 5.5422   -2.6681]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.5204    1.7930 ; 4.2846    2.1511 ; 3.8414    1.8848 ; 4.1144    1.4690 ; 4.5204    1.7930]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.1282   -0.3254 ; 3.9605    0.2413 ; 3.6270    0.1243 ; 3.4902   -0.1367 ; 3.7079   -0.5960 ; 4.1154   -0.4591 ; 4.1282   -0.3254]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % THREE CHAIRS LADDER
        case 21
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.9981   -2.2790 ; 4.9436   -1.8460 ; 4.3897   -1.9456 ; 4.4910   -2.3852 ; 4.9981   -2.2790]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.4193    1.6258 ; 4.5028    1.5748 ; 4.8311    1.9755 ; 4.5168    2.2783 ; 4.1432    1.9183 ; 4.4193    1.6258]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.2877    0.4720 ; 4.4343    0.9823 ; 3.7211    1.1996 ; 3.5636    0.6895 ; 4.2877    0.4720]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [4.0553   -0.2848 ; 3.8264    0.1809 ; 3.3254   -0.0364 ; 3.6138   -0.4816 ; 4.0553   -0.2848]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN 1
        case 22
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.5025    1.4895 ; 3.3274    1.8892 ; 2.8448    1.6987 ; 3.0463    1.2439 ; 3.5025    1.4895]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [2.6052   -1.6463 ; 2.7524   -1.4520 ; 2.6336   -1.2006 ; 2.4252   -1.5251 ; 2.6052   -1.6463]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.4430   -0.0166 ; 4.0462    0.1640 ; 3.8155   -0.2999 ; 4.2688   -0.5038 ; 4.4421   -0.0264 ; 4.4430   -0.0166]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [4.2205   -0.8925 ; 4.2036   -0.6493 ; 4.1060   -0.5231 ; 4.0502   -0.9142 ; 4.2205   -0.8925]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [5.6747    0.8151 ; 5.9204    1.2858 ; 5.2655    1.6409 ; 5.0088    1.1727 ; 5.6747    0.8151]';
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{5}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN 3
        case 23
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.2924    1.4418 ; 3.3527    1.4622 ; 3.2878    1.8928 ; 2.7736    1.8375 ; 2.6862    1.7147 ; 2.7445    1.3612 ; 2.8496    1.3468 ; 3.2924    1.4418]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.0511   -2.0478 ; 3.2845   -1.7080 ; 2.8569   -1.3747 ; 2.5680   -1.7793 ; 3.0511   -2.0478]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [3.3101   -0.3361 ; 3.2431   -0.1023 ; 3.0086   -0.0172 ; 3.1125   -0.4005 ; 3.3101   -0.3361]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [5.4313    0.5696 ; 5.4079    1.7143 ; 4.8676    1.7247 ; 4.8862    0.5201 ; 5.4313    0.5696]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [5.1648   -1.7359 ; 5.2438   -0.5926 ; 4.7507   -0.5288 ; 4.6598   -1.7316 ; 5.1648   -1.7359]';
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{5}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN 4    
        case 24
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.5670   -1.0174 ; 3.2720   -0.5111 ; 2.8514   -0.7746 ; 3.2216   -1.2417 ; 3.5670   -1.0174]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.5228    1.2882 ; 3.4621    2.5521 ; 3.0704    2.4849 ; 2.9654    2.3794 ; 2.9925    1.3206 ; 3.5228    1.2882]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [6.1967    0.8310 ; 6.2970    1.2283 ; 5.7432    1.4302 ; 5.6143    0.9523 ; 6.1967    0.8310]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [4.2900    0.1100 ; 4.1846    0.4808 ; 4.1058    0.3387 ; 4.1161    0.0989 ; 4.2900    0.1100]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [5.4099   -1.6858 ; 5.9801   -0.6920 ; 5.5521   -0.4166 ; 5.4990   -0.4644 ; 4.9456   -1.4589 ; 5.4099   -1.6858]';
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{5}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN 5
        case 25
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [2.7777   -1.1980 ; 3.0031   -1.3772 ; 3.2990   -1.0953 ; 2.9249   -0.6210 ; 2.5624   -0.9565 ; 2.7777   -1.1980]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.5139    1.1908 ; 3.4149    2.3298 ; 3.2590    2.4147 ; 2.9469    2.3428 ; 2.8859    2.2413 ; 2.9783    1.1841 ; 3.5139    1.1908]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.0539   -0.2465 ; 4.1534    0.1388 ; 3.9922    0.0416 ; 3.8873   -0.1736 ; 4.0539   -0.2465]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [3.7584   -0.7423 ; 3.8399   -0.5178 ; 3.7491   -0.4759 ; 3.7398   -0.4779 ; 3.6649   -0.6988 ; 3.7584   -0.7423]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [5.6946    0.5278 ; 6.0103    0.7424 ; 5.7759    1.1539 ; 5.3193    0.9117 ; 5.5625    0.4834; 5.6946    0.5278]';
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{6}.Points = [5.3484   -1.9827 ; 5.8759   -0.9671 ; 5.6972   -0.7832 ; 5.4574   -0.6983 ; 4.8925   -1.7044 ; 5.3484   -1.9827]';
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 5;
            Scenario.Map.Obstacle{5}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 6;
            Scenario.Map.Obstacle{6}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{6}.Rotation = 0;

%             Scenario.Map.Obstacle{1}.Type = 1;
%             Scenario.Map.Obstacle{1}.StartingPoint = [0,-0.2];
%             Scenario.Map.Obstacle{1}.Rotation = 0;
%             
%             Scenario.Map.Obstacle{2}.Type = 1;
%             Scenario.Map.Obstacle{2}.StartingPoint = [0,-4.4];
%             Scenario.Map.Obstacle{2}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN UNKNOWN - INITIAL POSITION 1
        case 26
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [2.4980   -1.4644 ; 2.1794   -1.0552 ; 2.0581   -1.0677 ; 1.7873   -1.3025 ; 2.1648   -1.7452 ; 2.4980   -1.4644]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.3495   -0.0051 ; 3.2084    0.1921 ; 3.0420    0.2723 ; 3.2070   -0.1053 ; 3.3495   -0.0051]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [5.6212   -1.2004 ; 5.6095   -0.0555 ; 5.4187    0.0112 ; 5.1302   -0.0174 ; 5.0754   -0.1045 ; 5.0867   -1.1674 ; 5.1343   -1.2230 ; 5.6212   -1.2004]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [3.5360    1.8747 ; 3.2926    2.2040 ; 2.8619    1.9182 ; 3.1548    1.5165 ; 3.5360    1.8747]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{5}.Type = 0;
            Scenario.Map.Obstacle{5}.Points = obstacle2D_ball([1.6950,0.6700],0.075,100);
            
            Scenario.Map.Obstacle{6}.Type = 0;
            Scenario.Map.Obstacle{6}.Points = obstacle2D_ball([4.2330,1.0170],0.075,100);
            
            Scenario.Map.Obstacle{7}.Type = 0;
            Scenario.Map.Obstacle{7}.Points = obstacle2D_ball([5.0410,2.1240],0.075,100);
            
            Scenario.Map.Obstacle{8}.Type = 0;
            Scenario.Map.Obstacle{8}.Points = obstacle2D_ball([3.4600,-1.6550],0.075,100);
            
            Scenario.Map.Obstacle{9}.Type = 0;
            Scenario.Map.Obstacle{9}.Points = obstacle2D_ball([6.1980,0.9560],0.18,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0 ; 4.328-3.407 1.853-0.039 0 ; 3.665-3.407 -1.628-0.039 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3 3 3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN UNKNOWN - INITIAL POSITION 2    
        case 27
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [2.4980   -1.4644 ; 2.1794   -1.0552 ; 2.0581   -1.0677 ; 1.7873   -1.3025 ; 2.1648   -1.7452 ; 2.4980   -1.4644]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.3495   -0.0051 ; 3.2084    0.1921 ; 3.0420    0.2723 ; 3.2070   -0.1053 ; 3.3495   -0.0051]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [5.6212   -1.2004 ; 5.6095   -0.0555 ; 5.4187    0.0112 ; 5.1302   -0.0174 ; 5.0754   -0.1045 ; 5.0867   -1.1674 ; 5.1343   -1.2230 ; 5.6212   -1.2004]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [3.5360    1.8747 ; 3.2926    2.2040 ; 2.8619    1.9182 ; 3.1548    1.5165 ; 3.5360    1.8747]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{5}.Type = 0;
            Scenario.Map.Obstacle{5}.Points = obstacle2D_ball([1.6950,0.6700],0.075,100);
            
            Scenario.Map.Obstacle{6}.Type = 0;
            Scenario.Map.Obstacle{6}.Points = obstacle2D_ball([4.2330,1.0170],0.075,100);
            
            Scenario.Map.Obstacle{7}.Type = 0;
            Scenario.Map.Obstacle{7}.Points = obstacle2D_ball([5.0410,2.1240],0.075,100);
            
            Scenario.Map.Obstacle{8}.Type = 0;
            Scenario.Map.Obstacle{8}.Points = obstacle2D_ball([3.4600,-1.6550],0.075,100);
            
            Scenario.Map.Obstacle{9}.Type = 0;
            Scenario.Map.Obstacle{9}.Points = obstacle2D_ball([6.1980,0.9560],0.18,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % MULTIPLE KNOWN UNKNOWN - INITIAL POSITION 3    
        case 28
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [2.4980   -1.4644 ; 2.1794   -1.0552 ; 2.0581   -1.0677 ; 1.7873   -1.3025 ; 2.1648   -1.7452 ; 2.4980   -1.4644]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [3.3495   -0.0051 ; 3.2084    0.1921 ; 3.0420    0.2723 ; 3.2070   -0.1053 ; 3.3495   -0.0051]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [5.6212   -1.2004 ; 5.6095   -0.0555 ; 5.4187    0.0112 ; 5.1302   -0.0174 ; 5.0754   -0.1045 ; 5.0867   -1.1674 ; 5.1343   -1.2230 ; 5.6212   -1.2004]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [3.5360    1.8747 ; 3.2926    2.2040 ; 2.8619    1.9182 ; 3.1548    1.5165 ; 3.5360    1.8747]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{5}.Type = 0;
            Scenario.Map.Obstacle{5}.Points = obstacle2D_ball([1.6950,0.6700],0.075,100);
            
            Scenario.Map.Obstacle{6}.Type = 0;
            Scenario.Map.Obstacle{6}.Points = obstacle2D_ball([4.2330,1.0170],0.075,100);
            
            Scenario.Map.Obstacle{7}.Type = 0;
            Scenario.Map.Obstacle{7}.Points = obstacle2D_ball([5.0410,2.1240],0.075,100);
            
            Scenario.Map.Obstacle{8}.Type = 0;
            Scenario.Map.Obstacle{8}.Points = obstacle2D_ball([3.4600,-1.6550],0.075,100);
            
            Scenario.Map.Obstacle{9}.Type = 0;
            Scenario.Map.Obstacle{9}.Points = obstacle2D_ball([6.1980,0.9560],0.18,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
        
        % SEMANTIC ALIGNMENT 1
        case 29
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [1.7980    0.6168 ; 1.8367    1.0511 ; 1.3223    1.1170 ; 1.2473    1.0314 ; 1.2203    0.6738 ; 1.7980    0.6168]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [2.6098   -1.3916 ; 2.2429   -0.9365 ; 1.9195   -1.2540 ; 2.2562   -1.6475 ; 2.6098   -1.3916]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [4.2340   -0.2518 ; 4.2370   -0.0082 ; 4.1167    0.1299 ; 4.0501   -0.2583 ; 4.2340   -0.2518]';
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [5.6993    1.8306 ; 5.1934    2.4550 ; 5.0641    2.4909 ; 4.6251    2.1334 ; 5.1819    1.4627 ; 5.6993    1.8306]';
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 3;
            Scenario.Map.Obstacle{3}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 4;
            Scenario.Map.Obstacle{4}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % SEMANTIC ALIGNMENT 2    
        case 30
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.2424   -0.2828 ; 3.3388   -0.1385 ; 3.1406   -0.0045 ; 2.9219    0.0141 ; 3.0519   -0.1443 ; 3.2424   -0.2828]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 1
        case 31
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.2424   -0.2828 ; 3.3388   -0.1385 ; 3.1406   -0.0045 ; 2.9219    0.0141 ; 3.0519   -0.1443 ; 3.2424   -0.2828]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [50,50];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 2
        case 32
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [4.0767  1.2022 ; 4.3643    1.0111 ; 4.6164    1.4630 ; 4.2540    1.7062 ; 3.9500    1.2864 ; 4.0767    1.2022]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 3
        case 33
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.8435    0.8554 ; 4.1014    1.3039 ; 3.7424    1.5518 ; 3.4330    1.1361 ; 3.8435    0.8554]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{2}.Points = [4.3471   -0.7476 ; 4.0190   -0.3545 ; 3.9529   -0.4180 ; 3.7638   -0.3587 ; 3.5111   -0.6065 ; 4.0360   -1.0444 ; 4.3471   -0.7476]';
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 2;
            Scenario.Map.Obstacle{2}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 4   
        case 34
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.1327    1.0900 ; 3.6863    1.8195 ; 4.3613    1.3516 ; 4.2408    1.1522 ; 4.7765   -0.3071 ; 4.4108   -0.4408 ; 4.6814   -0.8783 ; 4.0066   -1.3324 ; 3.5404   -0.7480 ; 3.4816   -0.3881 ; 3.7056   -0.2330 ; 3.3479    0.7242 ; 3.4687    0.8539 ; 3.1327    1.0900]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,0.0,'JointType','miter'); % ALREADY DILATED
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 5
        case 35
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.2799    1.1416 ; 3.8436    1.8668 ; 4.5019    1.3894 ; 4.3332    1.1096 ; 4.5253    1.1308 ; 4.8001   -0.3648 ; 4.5017   -0.4379 ; 4.7204   -0.8159 ; 4.0437   -1.3532 ; 3.4229   -0.7067 ; 3.8437   -0.3265 ; 3.6325    0.8185 ; 3.6649    0.8651 ; 3.2799    1.1416]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin, 0.0,'JointType','miter'); % ALREADY DILATED
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
        % HYBRID SYSTEM ILLUSTRATION - POSITION 6
        case 36
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.1;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.5;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 0.7;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.5;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [-1 -3; 7 -3; 7 3; -1 3];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 1.0;
            Scenario.Map.DiffeoParams.varepsilon = 1.0;
            Scenario.Map.DiffeoParams.mu1 = 2.0; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            Scenario.Map.KnownObstacle{1}.Points = [3.2799    1.1416 ; 3.8436    1.8668 ; 4.5019    1.3894 ; 4.3332    1.1096 ; 4.5253    1.1308 ; 4.8001   -0.3648 ; 4.5017   -0.4379 ; 4.7204   -0.8159 ; 4.0437   -1.3532 ; 3.4229   -0.7067 ; 3.8437   -0.3265 ; 3.6325    0.8185 ; 3.6649    0.8651 ; 3.2799    1.1416]';
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin, 0.0,'JointType','miter'); % ALREADY DILATED
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];
            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
%             Scenario.Map.Obstacle{1}.Type = 0;
%             Scenario.Map.Obstacle{1}.Points = obstacle2D_ball([4.169613, 0.510077956], 0.1820085, 100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0];
            Scenario.Start = [0 0 0];
            Scenario.Goal = [6.513,0];
            Scenario.Type = [1];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 250];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
        
        
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
        % RRT Comparison 1
        case 37
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 1.0;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 1.0;
            Scenario.Robot.LinearCmdMax = 1.0;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.5;
            Scenario.Map.DiffeoParams.varepsilon = 0.5;
            Scenario.Map.DiffeoParams.mu1 = 1; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            gap_radius = 0.39;
            Scenario.Map.KnownObstacle{1}.Points = [-3,-3,3,3,-3 ; 3.0-0.5*gap_radius,-3.0+0.5*gap_radius,-3.0+0.5*gap_radius,3.0-0.5*gap_radius,3.0-0.5*gap_radius];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            Scenario.Map.KnownObstacle{2}.Points = 0.5*[0,-1,-3,-1.6,-1.9,0,1.9,1.6,3,1,0; 3.33,1.43,1.03,-0.57,-2.67,-1.77,-2.67,-0.57,1.03,1.43,3.33];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{3}.Points = [-2,-2,2,2,2/3,-2/3,-2 ; 1.4,-0.6,-0.6,1.4,0.65,0.65,1.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{4}.Points = [-3,-3,3,3,2,2,-2,-2,-3 ; 3,0,0,3,3,1,1,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{5}.Points = [-3,-3,3,3,-3 ; 3,0,0,3,3];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{6}.Points = [2.518, 1.83, 2.043, 2.406, 2.655, 2.518 ; 0.5048, 0.2963, -0.2348, -0.8039, -0.0533, 0.5048];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            Scenario.Map.KnownObstacle{7}.Points = [-1,1,1,-1,-1 ; -0.5,-0.5,0.5,0.5,-0.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 1;
            Scenario.Map.Obstacle{1}.StartingPoint = [6,9.0+0.5*gap_radius];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 1;
            Scenario.Map.Obstacle{2}.StartingPoint = [6,3.0-0.5*gap_radius];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
            Scenario.Start = [1 10 0];
%             Scenario.Start = [2 5 0 ; 4 5 0 ; 2 8 0 ; 2 2 0 ; 5 9 0 ; 5 1 0];
            Scenario.Goal = [11, 10];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 400];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
            
        % Room layout
        case 38
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 1.0;
            Scenario.Robot.LinearCtrlGain = 1.0;
            Scenario.Robot.AngularCmdMax = 2.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 10; 15 10; 15 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.7;
            Scenario.Map.DiffeoParams.varepsilon = 0.7;
            Scenario.Map.DiffeoParams.mu1 = 0.8; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.1; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 1.80*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 1.5*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 2*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 2.5*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            % Wall 1
            Scenario.Map.KnownObstacle{6}.Points = [0.0,0.5,0.5,0.0,0.0 ; 1.9,1.9,2.1,2.1,1.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            % Wall 2
            Scenario.Map.KnownObstacle{7}.Points = [2.5,2.9,2.9,3.1,3.1,4.9,4.9,5.1,5.1,4.9,4.9,2.5,2.5 ; 1.9,1.9,0.0,0.0,1.9,1.9,1.5,1.5,2.5,2.5,2.1,2.1,1.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];
            
            % Wall 3
            Scenario.Map.KnownObstacle{8}.Points = [4.9,5.1,5.1,4.9,4.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{8}.Points(1,:),Scenario.Map.KnownObstacle{8}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{8}.DilatedPoints = [x2';y2'];
            
            % Wall 4
            Scenario.Map.KnownObstacle{9}.Points = [4.9,5.1,5.1,4.9,4.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{9}.Points(1,:),Scenario.Map.KnownObstacle{9}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{9}.DilatedPoints = [x2';y2'];
            
            % Wall 5
            Scenario.Map.KnownObstacle{9}.Points = [0.0,4.9,4.9,5.1,5.1,4.9,4.9,0.0,0.0 ; 3.9,3.9,3.5,3.5,6.0,6.0,4.1,4.1,3.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{9}.Points(1,:),Scenario.Map.KnownObstacle{9}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{9}.DilatedPoints = [x2';y2'];
            
            % Wall 6
            Scenario.Map.KnownObstacle{10}.Points = [10.9,10.9,9.9,9.9,11.5,11.5,11.1,11.1,10.9 ; 15.0,6.1,6.1,5.9,5.9,6.1,6.1,10.0,10.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{10}.Points(1,:),Scenario.Map.KnownObstacle{10}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{10}.DilatedPoints = [x2';y2'];
            
            % Wall 7
            Scenario.Map.KnownObstacle{11}.Points = [13.0,13.0,15.0,15.0,12.5 ; 6.1,5.9,5.9,6.1,6.1];
            polyin = polyshape(Scenario.Map.KnownObstacle{11}.Points(1,:),Scenario.Map.KnownObstacle{11}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{11}.DilatedPoints = [x2';y2'];
            
            % Wall 8
            Scenario.Map.KnownObstacle{12}.Points = [7.1,7.1,8.9,8.9,9.1,9.1,7.1,7.1,9.1,9.1,6.9,6.9; 0.0,1.9,1.9,1.5,1.5,2.1,2.1,3.9,3.9,4.1,4.1,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{12}.Points(1,:),Scenario.Map.KnownObstacle{12}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{12}.DilatedPoints = [x2';y2'];
            
            % Wall 9
            Scenario.Map.KnownObstacle{13}.Points = [8.9,9.1,9.1,8.9,8.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{13}.Points(1,:),Scenario.Map.KnownObstacle{13}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{13}.DilatedPoints = [x2';y2'];
            
            % Wall 10
            Scenario.Map.KnownObstacle{14}.Points = [11.9,12.1,12.1,12.5,12.5,10.5,10.5,11.9,11.9 ; 0.0,0.0,3.9,3.9,4.1,4.1,3.9,3.9,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{14}.Points(1,:),Scenario.Map.KnownObstacle{14}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{14}.DilatedPoints = [x2';y2'];
            
            % Wall 11
            Scenario.Map.KnownObstacle{15}.Points = [14.0,15.0,15.0,14.0,14.0 ; 3.9,3.9,4.1,4.1,3.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{15}.Points(1,:),Scenario.Map.KnownObstacle{15}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{15}.DilatedPoints = [x2';y2'];
            
            % Kitchen
            Scenario.Map.KnownObstacle{16}.Points = [1.0,0.0,0.0,5,5,4,4,1,1 ; 7,7,4,4,6,6,5,5,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{16}.Points(1,:),Scenario.Map.KnownObstacle{16}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{16}.DilatedPoints = [x2';y2'];
            
            % Kitchen
            Scenario.Map.KnownObstacle{16}.Points = [1.0,0.0,0.0,5,5,4,4,1,1 ; 7,7,4,4,6,6,5,5,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{16}.Points(1,:),Scenario.Map.KnownObstacle{16}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{16}.DilatedPoints = [x2';y2'];
            
            % Bed 1
            Scenario.Map.KnownObstacle{17}.Points = [12,12,14,14,12 ; 10,7,7,10,10];
            polyin = polyshape(Scenario.Map.KnownObstacle{17}.Points(1,:),Scenario.Map.KnownObstacle{17}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{17}.DilatedPoints = [x2';y2'];
            
            % Bed table
            Scenario.Map.KnownObstacle{18}.Points = 1.8*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{18}.Points(1,:),Scenario.Map.KnownObstacle{18}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{18}.DilatedPoints = [x2';y2'];
            
            % Closet 1
            Scenario.Map.KnownObstacle{19}.Points = [0.4,2.6,2.6,0.4,0.4 ; 0,0,1,1,0];
            polyin = polyshape(Scenario.Map.KnownObstacle{19}.Points(1,:),Scenario.Map.KnownObstacle{19}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{19}.DilatedPoints = [x2';y2'];
            
            % Desk 1
            Scenario.Map.KnownObstacle{20}.Points = [14,15,15,14,14 ; 1,1,3,3,1];
            polyin = polyshape(Scenario.Map.KnownObstacle{20}.Points(1,:),Scenario.Map.KnownObstacle{20}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{20}.DilatedPoints = [x2';y2'];
            
            % Bed 2
            Scenario.Map.KnownObstacle{21}.Points = [10,12,12,10,10 ; 0,0,2.7,2.7,0];
            polyin = polyshape(Scenario.Map.KnownObstacle{21}.Points(1,:),Scenario.Map.KnownObstacle{21}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{21}.DilatedPoints = [x2';y2'];
            
            % TV
            Scenario.Map.KnownObstacle{22}.Points = [10.7,11,11,10.7,10.7 ; 7,7,9,9,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{22}.Points(1,:),Scenario.Map.KnownObstacle{22}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{22}.DilatedPoints = [x2';y2'];
            
            % Closet 2
            Scenario.Map.KnownObstacle{23}.Points = [7.1,7.7,7.7,7.1,7.1 ; 2.1,2.1,3.9,3.9,2.1];
            polyin = polyshape(Scenario.Map.KnownObstacle{23}.Points(1,:),Scenario.Map.KnownObstacle{23}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{23}.DilatedPoints = [x2';y2'];
            
            % Washbasin
            Scenario.Map.KnownObstacle{24}.Points = [3.5,4.5,4.6,3.4,3.5 ; 1.5,1.5,1.9,1.9,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{24}.Points(1,:),Scenario.Map.KnownObstacle{24}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{24}.DilatedPoints = [x2';y2'];
            
            % Toilet
            angles = [-pi/2:0.1:pi/2]';
            points_t = [3.1 0.3 ; 3.6+0.3*cos(angles) 0.6+0.3*sin(angles) ; 3.6 0.9 ; 3.1 0.9 ; 3.1 0.3];
            Scenario.Map.KnownObstacle{25}.Points = points_t';
            polyin = polyshape(Scenario.Map.KnownObstacle{25}.Points(1,:),Scenario.Map.KnownObstacle{25}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{25}.DilatedPoints = [x2';y2'];
            
            % Desk 2
            Scenario.Map.KnownObstacle{26}.Points = [0,0.5,0.5,0,0 ; 2.3,2.3,3.7,3.7,2.3];
            polyin = polyshape(Scenario.Map.KnownObstacle{26}.Points(1,:),Scenario.Map.KnownObstacle{26}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{26}.DilatedPoints = [x2';y2'];
            
            % Wall 11
            Scenario.Map.KnownObstacle{27}.Points = [6.9,8.6,8.6,7.1,7.1,6.9,6.9 ; 5.9,5.9,6.1,6.1,10.0,10.0,5.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{27}.Points(1,:),Scenario.Map.KnownObstacle{27}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{27}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 6;
            Scenario.Map.Obstacle{1}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 7;
            Scenario.Map.Obstacle{2}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 8;
            Scenario.Map.Obstacle{3}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 9;
            Scenario.Map.Obstacle{4}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 10;
            Scenario.Map.Obstacle{5}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 11;
            Scenario.Map.Obstacle{6}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{6}.Rotation = 0;
            
            Scenario.Map.Obstacle{7}.Type = 12;
            Scenario.Map.Obstacle{7}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{7}.Rotation = 0;
            
            Scenario.Map.Obstacle{8}.Type = 13;
            Scenario.Map.Obstacle{8}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{8}.Rotation = 0;
            
            Scenario.Map.Obstacle{9}.Type = 14;
            Scenario.Map.Obstacle{9}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{9}.Rotation = 0;
            
            Scenario.Map.Obstacle{10}.Type = 15;
            Scenario.Map.Obstacle{10}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{10}.Rotation = 0;
            
            Scenario.Map.Obstacle{11}.Type = 4;
            Scenario.Map.Obstacle{11}.StartingPoint = [3.0,8.0];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{12}.Type = 3;
            Scenario.Map.Obstacle{12}.StartingPoint = [4.4,8.0];
            Scenario.Map.Obstacle{12}.Rotation = 0;
            
            Scenario.Map.Obstacle{13}.Type = 3;
            Scenario.Map.Obstacle{13}.StartingPoint = [2.5,7.0];
            Scenario.Map.Obstacle{13}.Rotation = 0;
            
            Scenario.Map.Obstacle{14}.Type = 3;
            Scenario.Map.Obstacle{14}.StartingPoint = [3.5,7.0];
            Scenario.Map.Obstacle{14}.Rotation = 0;
            
            Scenario.Map.Obstacle{15}.Type = 3;
            Scenario.Map.Obstacle{15}.StartingPoint = [2.5,9.0];
            Scenario.Map.Obstacle{15}.Rotation = 0;
            
            Scenario.Map.Obstacle{16}.Type = 3;
            Scenario.Map.Obstacle{16}.StartingPoint = [3.5,9.0];
            Scenario.Map.Obstacle{16}.Rotation = 0;
            
            Scenario.Map.Obstacle{17}.Type = 3;
            Scenario.Map.Obstacle{17}.StartingPoint = [1.6,8.0];
            Scenario.Map.Obstacle{17}.Rotation = 0;
            
            Scenario.Map.Obstacle{18}.Type = 1;
            Scenario.Map.Obstacle{18}.StartingPoint = [8.0,7.5];
            Scenario.Map.Obstacle{18}.Rotation = 0;
            
            Scenario.Map.Obstacle{19}.Type = 5;
            Scenario.Map.Obstacle{19}.StartingPoint = [9.5,9.0];
            Scenario.Map.Obstacle{19}.Rotation = -2*pi/3;
            
            Scenario.Map.Obstacle{20}.Type = 16;
            Scenario.Map.Obstacle{20}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{20}.Rotation = 0;
            
            Scenario.Map.Obstacle{21}.Type = 17;
            Scenario.Map.Obstacle{21}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{21}.Rotation = 0;
            
            Scenario.Map.Obstacle{22}.Type = 18;
            Scenario.Map.Obstacle{22}.StartingPoint = [11.55,9.5];
            Scenario.Map.Obstacle{22}.Rotation = 0;
            
            Scenario.Map.Obstacle{23}.Type = 18;
            Scenario.Map.Obstacle{23}.StartingPoint = [14.45,9.5];
            Scenario.Map.Obstacle{23}.Rotation = 0;
            
            Scenario.Map.Obstacle{24}.Type = 19;
            Scenario.Map.Obstacle{24}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{24}.Rotation = 0;
            
            Scenario.Map.Obstacle{25}.Type = 20;
            Scenario.Map.Obstacle{25}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{25}.Rotation = 0;
            
            Scenario.Map.Obstacle{26}.Type = 3;
            Scenario.Map.Obstacle{26}.StartingPoint = [13.5,1.5];
            Scenario.Map.Obstacle{26}.Rotation = -pi/4;
            
            Scenario.Map.Obstacle{27}.Type = 21;
            Scenario.Map.Obstacle{27}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{27}.Rotation = 0;
            
            Scenario.Map.Obstacle{28}.Type = 22;
            Scenario.Map.Obstacle{28}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{28}.Rotation = 0;
            
            Scenario.Map.Obstacle{29}.Type = 23;
            Scenario.Map.Obstacle{29}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{29}.Rotation = 0;
            
            Scenario.Map.Obstacle{30}.Type = 24;
            Scenario.Map.Obstacle{30}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{30}.Rotation = 0;
            
            Scenario.Map.Obstacle{31}.Type = 25;
            Scenario.Map.Obstacle{31}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{31}.Rotation = 0;
            
            Scenario.Map.Obstacle{32}.Type = 26;
            Scenario.Map.Obstacle{32}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{32}.Rotation = 0;
            
            Scenario.Map.Obstacle{33}.Type = 3;
            Scenario.Map.Obstacle{33}.StartingPoint = [1,3];
            Scenario.Map.Obstacle{33}.Rotation = -pi/6;
            
            Scenario.Map.Obstacle{34}.Type = 27;
            Scenario.Map.Obstacle{34}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{34}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0 ; 10 8 0 ; 3 6 0 ; 4.5 1 0 ; 14.5 8.5 0 ; 13.5 2.5 pi ; 1.5 2.0 0];
            Scenario.Start = [1 9 0 ; 10 8 0 ; 3 6 0 ; 4.5 1 0 ; 14.5 8.5 0 ; 13.5 2.5 pi ; 1.5 2.0 0];
%             Scenario.Start = [9 11.7 0];
            Scenario.Goal = [8 1];
            Scenario.Type = [3 3 3 3 3 3 3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [1:10,34];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 2000];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
            
        % RRT Cycling
        case 39
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 1.0;
            Scenario.Robot.LinearCtrlGain = 0.2;
            Scenario.Robot.AngularCmdMax = 1.0;
            Scenario.Robot.LinearCmdMax = 1.0;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 24; 24 24; 24 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.5;
            Scenario.Map.DiffeoParams.varepsilon = 0.5;
            Scenario.Map.DiffeoParams.mu1 = 1; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            gap_radius = 0.30;
            num_obstacles  = 5;
            Scenario.Map.KnownObstacle{1}.Points = [-1.5,-1.5,1.5,1.5,-1.5 ; (24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles),-(24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles),-(24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles),(24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles),(24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles)];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            for j=1:num_obstacles
                Scenario.Map.Obstacle{j}.Type = 1;
                Scenario.Map.Obstacle{j}.StartingPoint = [12,j*2*gap_radius + (j-1)*2*(24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles) + (24-(num_obstacles+1)*2*gap_radius)/(2*num_obstacles)];
                Scenario.Map.Obstacle{j}.Rotation = 0;
            end
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
                        
            % Start & Goal Configurations and Robot Type for each simulation
            Scenario.Start = [1 12 0];
%             Scenario.Start = [2 5 0 ; 4 5 0 ; 2 8 0 ; 2 2 0 ; 5 9 0 ; 5 1 0];
            Scenario.Goal = [23, 12];
            Scenario.Type = [3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 400];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-2);
            
            
            
            
        % Room with known and unknown obstacles
        case 40
            % Robot
            Scenario.Robot.Radius = 0.25;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 0.4;
            Scenario.Robot.LinearCtrlGain = 0.4;
            Scenario.Robot.AngularCmdMax = 5.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-2;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 12; 12 12; 12 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.8;
            Scenario.Map.DiffeoParams.varepsilon = 0.8;
            Scenario.Map.DiffeoParams.mu1 = 1.4; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 2.4*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 2*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 3*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 4*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 4;
            Scenario.Map.Obstacle{1}.StartingPoint = [7,4];
            Scenario.Map.Obstacle{1}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{2}.Type = 3;
            Scenario.Map.Obstacle{2}.StartingPoint = [7,5.4];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 1;
            Scenario.Map.Obstacle{3}.StartingPoint = [10,8.7];
            Scenario.Map.Obstacle{3}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{4}.Type = 5;
            Scenario.Map.Obstacle{4}.StartingPoint = [3,3];
            Scenario.Map.Obstacle{4}.Rotation = pi/4;
            
            Scenario.Map.Obstacle{5}.Type = 2;
            Scenario.Map.Obstacle{5}.StartingPoint = [3.,8.7];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 3;
            Scenario.Map.Obstacle{6}.StartingPoint = [1.7,6.9];
            Scenario.Map.Obstacle{6}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{7}.Type = 3;
            Scenario.Map.Obstacle{7}.StartingPoint = [1.7,8.7];
            Scenario.Map.Obstacle{7}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{8}.Type = 3;
            Scenario.Map.Obstacle{8}.StartingPoint = [1.7,10.5];
            Scenario.Map.Obstacle{8}.Rotation = -pi/2;
            
            Scenario.Map.Obstacle{9}.Type = 3;
            Scenario.Map.Obstacle{9}.StartingPoint = [4.3,6.9];
            Scenario.Map.Obstacle{9}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{10}.Type = 3;
            Scenario.Map.Obstacle{10}.StartingPoint = [4.3,8.7];
            Scenario.Map.Obstacle{10}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{11}.Type = 3;
            Scenario.Map.Obstacle{11}.StartingPoint = [4.3,10.5];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{12}.Type = 0;
            Scenario.Map.Obstacle{12}.Points = obstacle2D_ball([5.3,1.7],0.5,100);
            
            Scenario.Map.Obstacle{13}.Type = 0;
            Scenario.Map.Obstacle{13}.Points = obstacle2D_ball([10,2],0.5,100);
            
            Scenario.Map.Obstacle{14}.Type = 0;
            Scenario.Map.Obstacle{14}.Points = obstacle2D_ball([6.5,7.2],0.5,100);
            
            Scenario.Map.Obstacle{15}.Type = 0;
            Scenario.Map.Obstacle{15}.Points = obstacle2D_ball([10,6],1,100);
            
            Scenario.Map.Obstacle{16}.Type = 0;
            Scenario.Map.Obstacle{16}.Points = obstacle2D_ball([1.5,4.5],0.7,100);
            
            Scenario.Map.Obstacle{17}.Type = 0;
            Scenario.Map.Obstacle{17}.Points = obstacle2D_ball([4,5],0.3,100);
            
            Scenario.Map.Obstacle{18}.Type = 0;
            Scenario.Map.Obstacle{18}.Points = obstacle2D_ball([6.2,9.5],0.5,100);
            
            Scenario.Map.Obstacle{19}.Type = 0;
            Scenario.Map.Obstacle{19}.Points = obstacle2D_ball([7.5,1.3],0.5,100);
            
            Scenario.Map.Obstacle{20}.Type = 0;
            Scenario.Map.Obstacle{20}.Points = obstacle2D_ball([8,10.5],0.3,100);
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 11 0 ; 6 11.7 0 ; 3.2 7.5 0 ; 1 3 0 ; 4.2 4 0 ; 10.3 7.8 0];
%             Scenario.Start = [6 11.7 0 ; 1 2 0 ; 0.5 9.5 0 ; 5.5 6 0 ; 4.2 4 0 ; 10 11.5 0];
            Scenario.Start = [6 11.7 0 ; 1 2 0 ; 0.5 9.5 0 ; 5.5 6 0 ; 4.2 4 0 ; 10 11.5 0];
            Scenario.Goal = [11.2 1];
            Scenario.Type = [2 2 2 2 2 2];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 2000];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-1);
            
            
            
        % Room layout used for plotting
        case 41
            % Robot
            Scenario.Robot.Radius = 0.2;
            Scenario.Robot.SmallRadius = 0.15;
            Scenario.Robot.BodyAngle = linspace(-pi, pi, numSampleObstacle)';  
            Scenario.Robot.WheelAngle = linspace(5*pi/6, pi/6, numSampleObstacle)'; 
            Scenario.Robot.Polygon = Scenario.Robot.Radius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.SmallPolygon = Scenario.Robot.SmallRadius*[cos(Scenario.Robot.BodyAngle), sin(Scenario.Robot.BodyAngle)];
            Scenario.Robot.CtrlGain = 0.4;
            Scenario.Robot.AngularCtrlGain = 1.0;
            Scenario.Robot.LinearCtrlGain = 1.0;
            Scenario.Robot.AngularCmdMax = 2.4;
            Scenario.Robot.LinearCmdMax = 0.4;
            Scenario.Robot.RelTol = 1e-1;
            Scenario.Robot.Camera.Range = 2.0;
            Scenario.Robot.Camera.LidarRange = Scenario.Robot.Camera.Range;
            Scenario.Robot.Camera.Infinity = 20;
            Scenario.Robot.Camera.MinAngle = -pi;
            Scenario.Robot.Camera.MaxAngle = pi;
            Scenario.Robot.Camera.NumSample = numSampleCamera;
            Scenario.Robot.Camera.Resolution = (Scenario.Robot.Camera.MaxAngle - Scenario.Robot.Camera.MinAngle)/(Scenario.Robot.Camera.NumSample - 1);
            Scenario.Robot.Camera.Angle = linspace(Scenario.Robot.Camera.MinAngle, Scenario.Robot.Camera.MaxAngle, Scenario.Robot.Camera.NumSample);
            
            % Map
            Scenario.Map.Boundary = [0 0; 0 10; 15 10; 15 0];
            
            % Diffeo parameters
            Scenario.Map.DiffeoParams.epsilon = 0.7;
            Scenario.Map.DiffeoParams.varepsilon = 0.7;
            Scenario.Map.DiffeoParams.mu1 = 1.5; % must be roughly equal to epsilon^2, so that the switch scales almost linearly with beta
            Scenario.Map.DiffeoParams.mu2 = 0.05; % must be small so that sigma reaches the value of sigma_beta almost immediately
            Scenario.Map.DiffeoParams.p = 20;
            
            % Register known obstacles(all vertices must be given CCW)
            % Sofa
            Scenario.Map.KnownObstacle{1}.Points = 1.80*[-0.25,-0.25,0.8,0.8,0.25,0.25,-0.25 ; 1.,-0.25,-0.25,0.25,0.25,1.,1.];
            polyin = polyshape(Scenario.Map.KnownObstacle{1}.Points(1,:),Scenario.Map.KnownObstacle{1}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{1}.DilatedPoints = [x2';y2'];

            % Table 1
            Scenario.Map.KnownObstacle{2}.Points = 3*[-0.25,-0.25,0.25,0.25,-0.25 ; 0.8,-0.8,-0.8,0.8,0.8];
            polyin = polyshape(Scenario.Map.KnownObstacle{2}.Points(1,:),Scenario.Map.KnownObstacle{2}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{2}.DilatedPoints = [x2';y2'];
            
            % Chair
            Scenario.Map.KnownObstacle{3}.Points = 1.5*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{3}.Points(1,:),Scenario.Map.KnownObstacle{3}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{3}.DilatedPoints = [x2';y2'];
            
            % Table 2
            Scenario.Map.KnownObstacle{4}.Points = 2*[-0.3,-0.3,-0.2,0.2,0.3,0.3,0.2,-0.2,-0.3 ; 0.4,-0.4,-0.5,-0.5,-0.4,0.4,0.5,0.5,0.4];
            polyin = polyshape(Scenario.Map.KnownObstacle{4}.Points(1,:),Scenario.Map.KnownObstacle{4}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{4}.DilatedPoints = [x2';y2'];
            
            % Armchair
            Scenario.Map.KnownObstacle{5}.Points = 2.5*[-0.15,-0.2,-0.2,0.2,0.2,0.15,-0.15 ; 0.15,-0.1,-0.2,-0.2,-0.1,0.15,0.15];
            polyin = polyshape(Scenario.Map.KnownObstacle{5}.Points(1,:),Scenario.Map.KnownObstacle{5}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{5}.DilatedPoints = [x2';y2'];
            
            % Wall 1
            Scenario.Map.KnownObstacle{6}.Points = [0.0,0.5,0.5,0.0,0.0 ; 1.9,1.9,2.1,2.1,1.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{6}.Points(1,:),Scenario.Map.KnownObstacle{6}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{6}.DilatedPoints = [x2';y2'];
            
            % Wall 2
            Scenario.Map.KnownObstacle{7}.Points = [2.5,2.9,2.9,3.1,3.1,4.9,4.9,5.1,5.1,4.9,4.9,2.5,2.5 ; 1.9,1.9,0.0,0.0,1.9,1.9,1.5,1.5,2.5,2.5,2.1,2.1,1.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{7}.Points(1,:),Scenario.Map.KnownObstacle{7}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{7}.DilatedPoints = [x2';y2'];
            
            % Wall 3
            Scenario.Map.KnownObstacle{8}.Points = [4.9,5.1,5.1,4.9,4.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{8}.Points(1,:),Scenario.Map.KnownObstacle{8}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{8}.DilatedPoints = [x2';y2'];
            
            % Wall 4
            Scenario.Map.KnownObstacle{9}.Points = [4.9,5.1,5.1,4.9,4.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{9}.Points(1,:),Scenario.Map.KnownObstacle{9}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{9}.DilatedPoints = [x2';y2'];
            
            % Wall 5
            Scenario.Map.KnownObstacle{9}.Points = [0.0,4.9,4.9,5.1,5.1,4.9,4.9,0.0,0.0 ; 3.9,3.9,3.5,3.5,6.0,6.0,4.1,4.1,3.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{9}.Points(1,:),Scenario.Map.KnownObstacle{9}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{9}.DilatedPoints = [x2';y2'];
            
            % Wall 6
            Scenario.Map.KnownObstacle{10}.Points = [10.9,10.9,9.9,9.9,11.5,11.5,11.1,11.1,10.9 ; 15.0,6.1,6.1,5.9,5.9,6.1,6.1,10.0,10.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{10}.Points(1,:),Scenario.Map.KnownObstacle{10}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{10}.DilatedPoints = [x2';y2'];
            
            % Wall 7
            Scenario.Map.KnownObstacle{11}.Points = [13.0,13.0,15.0,15.0,12.5 ; 6.1,5.9,5.9,6.1,6.1];
            polyin = polyshape(Scenario.Map.KnownObstacle{11}.Points(1,:),Scenario.Map.KnownObstacle{11}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{11}.DilatedPoints = [x2';y2'];
            
            % Wall 8
            Scenario.Map.KnownObstacle{12}.Points = [7.1,7.1,8.9,8.9,9.1,9.1,7.1,7.1,9.1,9.1,6.9,6.9; 0.0,1.9,1.9,1.5,1.5,2.1,2.1,3.9,3.9,4.1,4.1,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{12}.Points(1,:),Scenario.Map.KnownObstacle{12}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{12}.DilatedPoints = [x2';y2'];
            
            % Wall 9
            Scenario.Map.KnownObstacle{13}.Points = [8.9,9.1,9.1,8.9,8.9 ; 0.0,0.0,0.5,0.5,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{13}.Points(1,:),Scenario.Map.KnownObstacle{13}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{13}.DilatedPoints = [x2';y2'];
            
            % Wall 10
            Scenario.Map.KnownObstacle{14}.Points = [11.9,12.1,12.1,12.5,12.5,10.5,10.5,11.9,11.9 ; 0.0,0.0,3.9,3.9,4.1,4.1,3.9,3.9,0.0];
            polyin = polyshape(Scenario.Map.KnownObstacle{14}.Points(1,:),Scenario.Map.KnownObstacle{14}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{14}.DilatedPoints = [x2';y2'];
            
            % Wall 11
            Scenario.Map.KnownObstacle{15}.Points = [14.0,15.0,15.0,14.0,14.0 ; 3.9,3.9,4.1,4.1,3.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{15}.Points(1,:),Scenario.Map.KnownObstacle{15}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{15}.DilatedPoints = [x2';y2'];
            
            % Kitchen
            Scenario.Map.KnownObstacle{16}.Points = [1.0,0.0,0.0,5,5,4,4,1,1 ; 7,7,4,4,6,6,5,5,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{16}.Points(1,:),Scenario.Map.KnownObstacle{16}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{16}.DilatedPoints = [x2';y2'];
            
            % Kitchen
            Scenario.Map.KnownObstacle{16}.Points = [1.0,0.0,0.0,5,5,4,4,1,1 ; 7,7,4,4,6,6,5,5,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{16}.Points(1,:),Scenario.Map.KnownObstacle{16}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{16}.DilatedPoints = [x2';y2'];
            
            % Bed 1
            Scenario.Map.KnownObstacle{17}.Points = [12,12,14,14,12 ; 10,7,7,10,10];
            polyin = polyshape(Scenario.Map.KnownObstacle{17}.Points(1,:),Scenario.Map.KnownObstacle{17}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{17}.DilatedPoints = [x2';y2'];
            
            % Bed table
            Scenario.Map.KnownObstacle{18}.Points = 1.8*[-0.2,0.2,0.2,-0.2,-0.2 ; -0.2,-0.2,0.2,0.2,-0.2];
            polyin = polyshape(Scenario.Map.KnownObstacle{18}.Points(1,:),Scenario.Map.KnownObstacle{18}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{18}.DilatedPoints = [x2';y2'];
            
            % Closet 1
            Scenario.Map.KnownObstacle{19}.Points = [0.4,2.6,2.6,0.4,0.4 ; 0,0,1,1,0];
            polyin = polyshape(Scenario.Map.KnownObstacle{19}.Points(1,:),Scenario.Map.KnownObstacle{19}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{19}.DilatedPoints = [x2';y2'];
            
            % Desk 1
            Scenario.Map.KnownObstacle{20}.Points = [14,15,15,14,14 ; 1,1,3,3,1];
            polyin = polyshape(Scenario.Map.KnownObstacle{20}.Points(1,:),Scenario.Map.KnownObstacle{20}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{20}.DilatedPoints = [x2';y2'];
            
            % Bed 2
            Scenario.Map.KnownObstacle{21}.Points = [10,12,12,10,10 ; 0,0,2.7,2.7,0];
            polyin = polyshape(Scenario.Map.KnownObstacle{21}.Points(1,:),Scenario.Map.KnownObstacle{21}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{21}.DilatedPoints = [x2';y2'];
            
            % TV
            Scenario.Map.KnownObstacle{22}.Points = [10.7,11,11,10.7,10.7 ; 7,7,9,9,7];
            polyin = polyshape(Scenario.Map.KnownObstacle{22}.Points(1,:),Scenario.Map.KnownObstacle{22}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{22}.DilatedPoints = [x2';y2'];
            
            % Closet 2
            Scenario.Map.KnownObstacle{23}.Points = [7.1,7.7,7.7,7.1,7.1 ; 2.1,2.1,3.9,3.9,2.1];
            polyin = polyshape(Scenario.Map.KnownObstacle{23}.Points(1,:),Scenario.Map.KnownObstacle{23}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{23}.DilatedPoints = [x2';y2'];
            
            % Washbasin
            Scenario.Map.KnownObstacle{24}.Points = [3.5,4.5,4.6,3.4,3.5 ; 1.5,1.5,1.9,1.9,1.5];
            polyin = polyshape(Scenario.Map.KnownObstacle{24}.Points(1,:),Scenario.Map.KnownObstacle{24}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{24}.DilatedPoints = [x2';y2'];
            
            % Toilet
            angles = [-pi/2:0.1:pi/2]';
            points_t = [3.1 0.3 ; 3.6+0.3*cos(angles) 0.6+0.3*sin(angles) ; 3.6 0.9 ; 3.1 0.9 ; 3.1 0.3];
            Scenario.Map.KnownObstacle{25}.Points = points_t';
            polyin = polyshape(Scenario.Map.KnownObstacle{25}.Points(1,:),Scenario.Map.KnownObstacle{25}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{25}.DilatedPoints = [x2';y2'];
            
            % Desk 2
            Scenario.Map.KnownObstacle{26}.Points = [0,0.5,0.5,0,0 ; 2.3,2.3,3.7,3.7,2.3];
            polyin = polyshape(Scenario.Map.KnownObstacle{26}.Points(1,:),Scenario.Map.KnownObstacle{26}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{26}.DilatedPoints = [x2';y2'];
            
            % Wall 11
            Scenario.Map.KnownObstacle{27}.Points = [6.9,8.6,8.6,7.1,7.1,6.9,6.9 ; 5.9,5.9,6.1,6.1,10.0,10.0,5.9];
            polyin = polyshape(Scenario.Map.KnownObstacle{27}.Points(1,:),Scenario.Map.KnownObstacle{27}.Points(2,:));
            polyout = polybuffer(polyin,Scenario.Robot.Radius,'JointType','miter');
            x2 = flipud(polyout.Vertices(:,1)); x2 = [x2;x2(1)];
            y2 = flipud(polyout.Vertices(:,2)); y2 = [y2;y2(1)];
            Scenario.Map.KnownObstacle{27}.DilatedPoints = [x2';y2'];

            
            % Register obstacles (type 0 if they are unknown)
            Scenario.Map.Obstacle{1}.Type = 6;
            Scenario.Map.Obstacle{1}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{1}.Rotation = 0;
            
            Scenario.Map.Obstacle{2}.Type = 7;
            Scenario.Map.Obstacle{2}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{2}.Rotation = 0;
            
            Scenario.Map.Obstacle{3}.Type = 8;
            Scenario.Map.Obstacle{3}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{3}.Rotation = 0;
            
            Scenario.Map.Obstacle{4}.Type = 9;
            Scenario.Map.Obstacle{4}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{4}.Rotation = 0;
            
            Scenario.Map.Obstacle{5}.Type = 10;
            Scenario.Map.Obstacle{5}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{5}.Rotation = 0;
            
            Scenario.Map.Obstacle{6}.Type = 11;
            Scenario.Map.Obstacle{6}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{6}.Rotation = 0;
            
            Scenario.Map.Obstacle{7}.Type = 12;
            Scenario.Map.Obstacle{7}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{7}.Rotation = 0;
            
            Scenario.Map.Obstacle{8}.Type = 13;
            Scenario.Map.Obstacle{8}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{8}.Rotation = 0;
            
            Scenario.Map.Obstacle{9}.Type = 14;
            Scenario.Map.Obstacle{9}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{9}.Rotation = 0;
            
            Scenario.Map.Obstacle{10}.Type = 15;
            Scenario.Map.Obstacle{10}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{10}.Rotation = 0;
            
            Scenario.Map.Obstacle{11}.Type = 4;
            Scenario.Map.Obstacle{11}.StartingPoint = [3.0,8.0];
            Scenario.Map.Obstacle{11}.Rotation = pi/2;
            
            Scenario.Map.Obstacle{12}.Type = 3;
            Scenario.Map.Obstacle{12}.StartingPoint = [4.4,8.0];
            Scenario.Map.Obstacle{12}.Rotation = 0;
            
            Scenario.Map.Obstacle{13}.Type = 3;
            Scenario.Map.Obstacle{13}.StartingPoint = [2.5,7.0];
            Scenario.Map.Obstacle{13}.Rotation = 0;
            
            Scenario.Map.Obstacle{14}.Type = 3;
            Scenario.Map.Obstacle{14}.StartingPoint = [3.5,7.0];
            Scenario.Map.Obstacle{14}.Rotation = 0;
            
            Scenario.Map.Obstacle{15}.Type = 3;
            Scenario.Map.Obstacle{15}.StartingPoint = [2.5,9.0];
            Scenario.Map.Obstacle{15}.Rotation = 0;
            
            Scenario.Map.Obstacle{16}.Type = 3;
            Scenario.Map.Obstacle{16}.StartingPoint = [3.5,9.0];
            Scenario.Map.Obstacle{16}.Rotation = 0;
            
            Scenario.Map.Obstacle{17}.Type = 3;
            Scenario.Map.Obstacle{17}.StartingPoint = [1.6,8.0];
            Scenario.Map.Obstacle{17}.Rotation = 0;
            
            Scenario.Map.Obstacle{18}.Type = 1;
            Scenario.Map.Obstacle{18}.StartingPoint = [8.0,7.5];
            Scenario.Map.Obstacle{18}.Rotation = 0;
            
            Scenario.Map.Obstacle{19}.Type = 5;
            Scenario.Map.Obstacle{19}.StartingPoint = [9.5,9.0];
            Scenario.Map.Obstacle{19}.Rotation = -2*pi/3;
            
            Scenario.Map.Obstacle{20}.Type = 16;
            Scenario.Map.Obstacle{20}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{20}.Rotation = 0;
            
            Scenario.Map.Obstacle{21}.Type = 17;
            Scenario.Map.Obstacle{21}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{21}.Rotation = 0;
            
            Scenario.Map.Obstacle{22}.Type = 18;
            Scenario.Map.Obstacle{22}.StartingPoint = [11.55,9.5];
            Scenario.Map.Obstacle{22}.Rotation = 0;
            
            Scenario.Map.Obstacle{23}.Type = 18;
            Scenario.Map.Obstacle{23}.StartingPoint = [14.45,9.5];
            Scenario.Map.Obstacle{23}.Rotation = 0;
            
            Scenario.Map.Obstacle{24}.Type = 19;
            Scenario.Map.Obstacle{24}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{24}.Rotation = 0;
            
            Scenario.Map.Obstacle{25}.Type = 20;
            Scenario.Map.Obstacle{25}.StartingPoint = [0.0,0.0];
            Scenario.Map.Obstacle{25}.Rotation = 0;
            
            Scenario.Map.Obstacle{26}.Type = 3;
            Scenario.Map.Obstacle{26}.StartingPoint = [13.5,1.5];
            Scenario.Map.Obstacle{26}.Rotation = -pi/4;
            
            Scenario.Map.Obstacle{27}.Type = 21;
            Scenario.Map.Obstacle{27}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{27}.Rotation = 0;
            
            Scenario.Map.Obstacle{28}.Type = 22;
            Scenario.Map.Obstacle{28}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{28}.Rotation = 0;
            
            Scenario.Map.Obstacle{29}.Type = 23;
            Scenario.Map.Obstacle{29}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{29}.Rotation = 0;
            
            Scenario.Map.Obstacle{30}.Type = 24;
            Scenario.Map.Obstacle{30}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{30}.Rotation = 0;
            
            Scenario.Map.Obstacle{31}.Type = 25;
            Scenario.Map.Obstacle{31}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{31}.Rotation = 0;
            
            Scenario.Map.Obstacle{32}.Type = 26;
            Scenario.Map.Obstacle{32}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{32}.Rotation = 0;
            
            Scenario.Map.Obstacle{33}.Type = 3;
            Scenario.Map.Obstacle{33}.StartingPoint = [1,3];
            Scenario.Map.Obstacle{33}.Rotation = -pi/6;
            
            Scenario.Map.Obstacle{34}.Type = 27;
            Scenario.Map.Obstacle{34}.StartingPoint = [0,0];
            Scenario.Map.Obstacle{34}.Rotation = 0;
            
            
            % Populate obstacle points
            for i=1:numel(Scenario.Map.Obstacle)
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.Points(2,:)]';
                Scenario.Map.Obstacle{i}.Points = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.Points(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.Points(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.Points(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.Points(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(1,:);Scenario.Map.KnownObstacle{Scenario.Map.Obstacle{i}.Type}.DilatedPoints(2,:)]';
                Scenario.Map.Obstacle{i}.DilatedPoints = [Scenario.Map.Obstacle{i}.StartingPoint(1)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*cos(Scenario.Map.Obstacle{i}.Rotation)-Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*sin(Scenario.Map.Obstacle{i}.Rotation),Scenario.Map.Obstacle{i}.StartingPoint(2)+Scenario.Map.Obstacle{i}.DilatedPoints(:,1)*sin(Scenario.Map.Obstacle{i}.Rotation)+Scenario.Map.Obstacle{i}.DilatedPoints(:,2)*cos(Scenario.Map.Obstacle{i}.Rotation)];
            end
            
            Scenario.Map.Obstacle{35}.Type = 0;
            Scenario.Map.Obstacle{35}.Points = obstacle2D_ball([5.7,7],0.5,100);
            
            Scenario.Map.Obstacle{36}.Type = 0;
            Scenario.Map.Obstacle{36}.Points = obstacle2D_ball([7,5],0.3,100);
            
            Scenario.Map.Obstacle{36}.Type = 0;
            Scenario.Map.Obstacle{36}.Points = obstacle2D_ball([7,5],0.3,100);
            
            Scenario.Map.Obstacle{37}.Type = 0;
            Scenario.Map.Obstacle{37}.Points = obstacle2D_ball([4,3],0.2,100);
            
            Scenario.Map.Obstacle{38}.Type = 0;
            Scenario.Map.Obstacle{38}.Points = obstacle2D_ball([12.2,5.2],0.25,100);
            
            Scenario.Map.Obstacle{39}.Type = 0;
            Scenario.Map.Obstacle{39}.Points = obstacle2D_ball([9,3],0.25,100);
            
            Scenario.Map.Obstacle{40}.Type = 0;
            Scenario.Map.Obstacle{40}.Points = obstacle2D_ball([6,2],0.25,100);
            
            Scenario.Map.Obstacle{41}.Type = 0;
            Scenario.Map.Obstacle{41}.Points = obstacle2D_ball([1.5,1.8],0.3,100);
            
            Scenario.Map.Obstacle{42}.Type = 0;
            Scenario.Map.Obstacle{42}.Points = obstacle2D_ball([13,3],0.3,100);
            
                        
            % Start & Goal Configurations and Robot Type for each simulation
%             Scenario.Start = [1 9 0 ; 10 8 0 ; 3 6 0 ; 4.5 1 0 ; 14.5 8.5 0 ; 13.5 2.5 pi ; 1.5 2.0 0];
            Scenario.Start = [1 9 0];
%             Scenario.Start = [9 11.7 0];
            Scenario.Goal = [8 1];
            Scenario.Type = [2 3 3 3 3 3 3];
            
            % List of initially known obstacles
            Scenario.InitiallyKnown = [1:10,34];
            
            % RRT settings
            Scenario.RRT.Samples = 2000;
            Scenario.RRT.Epsilon = 0.3;
            Scenario.RRT.BallRadius = 0.8;
            Scenario.RRT.PathDiscretization = 0.01;
            Scenario.RRT.UpdateRate = 10;
            
            % ODE Settings
            Scenario.ODE.tspan = [0, 2000];
            Scenario.ODE.option = odeset('RelTol', 1e-5, 'AbsTol', 1e-4, 'MaxStep', 1e-1);


        otherwise % Unknown Scenario ID
            error('Matlab:scenario:UnknownScenarioID', 'Unknown Scenario ID');

    end 
       
end
























