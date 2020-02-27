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


% polyxray(x,y,b, v) computes the intersection of the boundary of a
% polygon, with vertex coordinates x and y, and a ray, defined by a base
% point b on the line and a direction vector v.  
%
% Input:
%   x, y : Vertex coordinates of a polygon
%   b, v : Ray parameters starting at the base point b and in the direction of v 
% Output:
%   cx, cy : Vertex coordinates of the intersecting points of the polygon
%            boundary and ray
% Usage:
%   x = [0 1 1 0];
%   y = [0 0 1 1];
%   b = [0.5 0.5];
%   v = [cos(pi/6), sin(pi/6)];
%   [cx, cy] = polyxray(x, y, b, v);
%   figure, hold on;
%   patch(x,y,'w');
%   scatter(cx, cy, 60, 'r');
%   axis equal;

function  [cx, cy] = polyxray(x, y, b, v)
% Author: Omur Arslan - omur@seas.upenn.edu
% Date: August 19th, 2015 Wednesday

% Find the intersection with the line containing the input ray
[cx, cy] = polyxline(x, y, b, [-v(2), v(1)]);
% Only return the intersection points on the ray
a = (cx - b(1))*v(1) + (cy - b(2))*v(2);

cx = cx(a>1e-5);
cy = cy(a>1e-5);
