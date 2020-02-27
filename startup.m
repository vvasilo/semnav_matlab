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


% This startup.m is required to ensure that all of the matlab code is
% correctly on the path. startup.m is run when Matlab is started
% from this directory.

%% Recursively adds anything under the 'code' directory to the path.
fprintf('Adding all subdirectories of current directory to path.\n');
addpath(genpath(pwd));

path_python_ubuntu = '/usr/bin/python3'; % Mac: /Users/vasilis_vasilop/miniconda3/bin/python3, Ubuntu: /usr/bin/python3
path_python_mac = '/Users/vasilis_vasilop/miniconda3/bin/python3'; % Mac: /Users/vasilis_vasilop/miniconda3/bin/python3, Ubuntu: /usr/bin/python3
path_semnav_ubuntu = '/home/kodlab/catkin_ws/src/semnav/src/libraries/'; % Mac: /Users/vasilis_vasilop/Documents/Kodlab/semnav/src/libraries, Ubuntu: /home/kodlab/catkin_ws/src/semnav/src/libraries/
path_semnav_mac = '/Users/vasilis_vasilop/Documents/Kodlab/semnav/src/libraries'; % Mac: /Users/vasilis_vasilop/Documents/Kodlab/semnav/src/libraries, Ubuntu: /home/kodlab/catkin_ws/src/semnav/src/libraries/
path_packages = '/Users/vasilis_vasilop/miniconda3/lib/python3.7/site-packages'; % ONLY Mac: /Users/vasilis_vasilop/miniconda3/lib/python3.7/site-packages

%% Add current folder to Python search path
[v, e, isloaded] = pyversion;
if ~isloaded
    if strcmp(computer, 'GLNXA64')
        pyversion(path_python_ubuntu)
    elseif strcmp(computer,'MACI64')
        pyversion(path_python_mac)
    end
end

% Set flags
flag = int32(bitor(2,8));
py.sys.setdlopenflags(flag);

% Add the path to the desired Python libraries for the reactive planner
if strcmp(computer, 'GLNXA64')
    if count(py.sys.path,path_semnav_ubuntu) == 0
        insert(py.sys.path,int32(0),path_semnav_ubuntu);
    end
elseif strcmp(computer,'MACI64')
    if count(py.sys.path,path_semnav_mac) == 0
        insert(py.sys.path,int32(0),path_semnav_mac);
    end
end

% Locate packages from their original folder
if strcmp(computer,'MACI64')
    if count(py.sys.path,path_packages) == 0
        insert(py.sys.path,int32(0),path_packages);
    end
end

% Import the reactive planner library module
py.importlib.import_module('reactive_planner_lib');
py.importlib.import_module('polygeom_lib');
