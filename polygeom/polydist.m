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


% Compute the distance between a set of points and a polygon, and return 
% the closest points on the polygon boundary.   
% Here, distance is defined as the minimum distance between an input point 
% and any point on the polygon boundary.
%
% Input:  
%    X, Y - point coordinates
%    XV, YV - coordinates of polygon vertices
% Output: 
%    D - distance between points and the polygon 
%    CX, CY: coordinates of the closest points on the polygon to the input points
% Usage:
%   n = 10;
%   v = 8;
%   X = 2 * rand(n,1) - 1;
%   Y = 2 * rand(n,1) - 1;
%   th = linspace(0, 2*pi, v+1)';
%   XV = 0.5*cos(th);
%   YV = 0.5*sin(th);
%   [D, CX, CY] = polydist(X, Y, XV, YV);
%   figure, hold on;
%   patch(XV, YV, 'y');
%   plot(X, Y, 'ro');
%   plot(CX, CY, 'rx');
%   plot([X CX]', [Y CY]', 'b');
%   axis equal;
%   axis([-1 1 -1 1]);
%   box on;
%   grid on;

function [D, CX, CY] = polydist(X, Y, XV, YV)
% Author(s): 
%   Omur Arslan         omur@seas.upenn.edu
%   Vincent Pacelli     pacelliv@seas.upenn.edu
% Date: October 30, 2016 

%Input validation
if ~isequal(size(X), size(Y))
    error('MATLAB:polydist:DimensionMismatch', 'Point Coordinates, X and Y, must have the same size!')
end
if ~isequal(size(XV), size(YV))
    error('MATLAB:polydist:DimensionMismatch', 'Polygon coordinates, XV and YV, must have the same size!');
end

% Distance to empty set is infinity
if isempty(XV)
   D = Inf;
   CX = Inf;
   CY = Inf;
   return;
end

% Remove leading singleton dimensions
[X, nshifts] = shiftdim(X);
Y = shiftdim(Y);
XV = shiftdim(XV);
YV = shiftdim(YV);
% Relative coordinates of polygon rims
dX = circshift(XV,1) - XV; 
dY = circshift(YV,1) - YV;

% Vectorization 
n = size(X,1);  % number of points
v = size(XV,1); % number of polygon vertices
MdX = repmat(dX', [n, 1]);
MdY = repmat(dY', [n, 1]);
MXV = repmat(XV', [n, 1]);
MYV = repmat(YV', [n, 1]);
MX = repmat(X, [1, v]);
MY = repmat(Y, [1, v]);

% Compute the convex combination coefficients that define the closest
% points on polygon rims and input points 
A = (MdX.*(MX - MXV) + MdY.*(MY - MYV))./(MdX.^2 + MdY.^2);
A = max(min(A,1),0); 

% Minimum distance to polygon rims 
MD = (A.*MdX + MXV - MX).^2 + (A.*MdY + MYV - MY).^2;
% Minimum distance to the polygon
[D, I] = min(MD, [], 2);
D = sqrt(D);
I = sub2ind([n, v], (1:n)', I);
% Closest points on the polygon boundary
CX = A(I).*MdX(I) + MXV(I);
CY = A(I).*MdY(I) + MYV(I);

% Adjust the sign of distance for points in the polygon
IN = inpolygon(X, Y, XV, YV); 
D(IN) = -D(IN);

% Restore orginal dimensions
CX = shiftdim(CX, -nshifts);
CY = shiftdim(CY, -nshifts);
D = shiftdim(D, -nshifts);