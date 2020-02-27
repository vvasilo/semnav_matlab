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


% A circular obstacle model of radius r centered at c. 
%
% Input:
%   c : ball center (1x2 row vector)
%   r : ball radius (1x1 scalar)
%   n : number of samples (1x1 scalar)
% Output:
%   X : sample vertex coordinates of the input ball (nx2 matrix) 
% Usage:
%   c = rand(1,2);
%   r = rand(1);
%   n = 20;
%   X = obstacle2D_ball(c, r, n);
%   figure; patch(X(:,1), X(:,2),'r'); axis equal; box on; grid on;

function X = obstacle2D_ball(c, r, n)

% Uniform angle samples on [0, 2*pi)
angle = linspace(0, 2*pi, n + 1); % angles in clockwise order
angle = flipud(angle(1:n)');
% Vertex coordinates of the ball centeret at c with radius r
X = [c(1) + r * cos(angle), c(2) + r * sin(angle)];
