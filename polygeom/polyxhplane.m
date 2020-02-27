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


% polyxhplane(x, y, m, n) computes the intersection of a polygon, with
% vertex coordinates x and y, and a halfplane, defined by a  boundary
% point m and the inward normal n. 
%
% Input:
%   x, y : the vertex coordinates of a polygon
%   m, n : a boundary point and the inward normal of a halfplane
% Output:
%   xi, yi : the vertex coordinates of their intersection
% Usage:
%   k = 3;
%   x = rand(k,1); y = rand(k,1);
%   m = rand(1,2); n = 2*rand(1,2)-1;
%   [xi, yi] = polyxhplane(x, y, m, n);

function [xi, yi] = polyxhplane(x, y, m, n)
% Author: Omur Arslan - omur@seas.upenn.edu
% Date: November 10, 2015 Tuesday

if numel(x) ~= numel(y)
    error('MATLAB:polyxhplane:DimensionMismatch','Dimension Mismatch! Coordinate arrays should have the same number of elements.');
end
if (numel(m) ~= 2)||(numel(n)~=2)
    error('MATLAB:polyxhplane:NotHalfPlane', 'The input parameters do not specify a halp plane!'); 
end

m = m(:)'; % The point of the halfplane boundary is a row vector
n = n(:); % The inward normal of the halfplane is a column vector

xi = [];
yi = [];
K = numel(x); % number of element
for ck = 1:K % Current vertex ID
    cn = mod(ck, K) + 1; % Next vertex ID
    
    % Current and next vertices and their perpendicular distance to the
    % separating hyperplane
    vk = [x(ck), y(ck)]; 
    vn = [x(cn), y(cn)]; 
    sk =  (vk - m)*n;
    sn =  (vn - m)*n;

    if (sk*sn) < 0
        % Compute the point on the boundary and include it into the final result
        w = ((m - vn)*n)/((vk - vn)*n);
        w = max(min(w, 1), 0); % the boundary point should be a convex combination of the associated vertices
        vi = w*vk + (1 - w)*vn;
        xi = [xi; vi(1)];
        yi = [yi; vi(2)];
    end

    if (sn >= 0)
        % Include the next vertex since it is on the proper sice
        % of the hyperplane
        xi = [xi; vn(1)];
        yi = [yi; vn(2)];
    end    
end
