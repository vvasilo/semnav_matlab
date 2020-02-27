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


% A star-shaped obstacle model based on quadric functions.
% The equation is: beta(x,y) = [x^2, y^2]*P*[x^2; y^2] - r
%
% Input:
%   c : ball center (1x2 row vector)
%   P : semi-definite matrix (2x2 matrix)
%   r : level set of quadric function (1x1 scalar)
%   n : number of samples (1x1 scalar)
% Output:
%   X : sample vertex coordinates of the input star (nx2 matrix) 
% Usage:
%   c = rand(1,2);
%   P = [8,-5;-8,6];
%   r = 15;
%   n = 200;
%   X = obstacle2D_star(c, P, r, n);
%   figure; patch(X(:,1), X(:,2),'r'); axis equal; box on; grid on;

function X = obstacle2D_star(c, P, r, n)

% Define angles
angle = linspace(0, 2*pi, n); % angles in clockwise order

% Find coordinates
for i = 1:length(angle)
    rho(i) = (r/(P(1,1)*cos(angle(i))^4+(P(1,2)+P(2,1))*cos(angle(i))^2*sin(angle(i))^2+P(2,2)*sin(angle(i))^4))^0.25;
end

% Get coordinates
x_data = [rho.*cos(angle)];
y_data = [rho.*sin(angle)];
X = transpose([c(1)+x_data; c(2)+y_data]);
