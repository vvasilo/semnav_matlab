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


% Dilation (expansion) of a convex polygon by a closed circle of radius R
%
% Input: 
%   X1, Y1  : Vertex Coordinates of a convex polygon
%   R       : Radius of the dilation (expansion) disk
% Output:
%   X2, Y2  : Coordinates of expanded convex polygon
% Usage:
%   X1 = rand(3,1);
%   Y1 = rand(3,1);
%   R = 0.2;
%   [X2, Y2] =cvxpolydilate(X1, Y1, R);
%   patch(X1, Y1, 'r', 'EdgeColor', 'r', 'FaceColor', 'r');
%   patch(X2, Y2, 'b', 'EdgeColor', 'b', 'FaceColor', 'none');


function [X2, Y2] = cvxpolydilate(X1, Y1, R)
% Author: Omur Arslan, omur@seas.upenn.edu
% Date: November 13, 2015

X1 = X1(:);
Y1 = Y1(:);

if (length(X1) ~= length(Y1))
    error('Dimension Mismatch! Vertices should have the same number of coordinates.');
end
if (R < 0)
    error('Dilation (expansion) radius should be nonnegative!');
end

N = length(X1); % Number of vertices
if (N < 2)
    X2 = X1;
    Y2 = Y1;
else
    % For nontrivial convex polygons: 
    % Determine if the vertices of the input polygon is in clockwise or
    % counter-clockwise order
    orientsign = 1 - 2 * ispolycw(X1, Y1);
    
    % Compute the expansion of the input polygon
    % Memory allocation and vertex counter for the expanded polygon
    Q = [X1, Y1]; % Vectorized representation of the input polygon
    P = zeros(2*N, 2); % Vectorized memory allocation for the output polygon
    cP = 0; % Vertex counter of the output polygon
    for ck = 1:N % Current vertex index
        cn = mod(ck,N) + 1; % Next vertex index        
        % Edge coordinates and length
        E = Q(ck,:) - Q(cn,:);  
        if (norm(E) > 0)
            % For any nontrivial edge  compute its normal to determine the amount of the required vertex shift 
            nk = orientsign * [-E(2), E(1)]/norm(E); % Edge Normal
            sk = R * nk; % Amount of vertex shift
            % Expand the polygon by properly shifting its edge vertices
            P(cP*2 + 1,:) = Q(ck,:) + sk;
            P(cP*2 + 2,:) = Q(cn,:) + sk;
            
%             figure; hold on;
%             patch(X1, Y1, 'r', 'EdgeColor', 'r');
%             plot(X1([ck, cn]), Y1([ck, cn]), 'b');
%             plot(P(cP*2 + [1 2], 1), P(cP*2 + [1 2], 2), 'g');
%             close all
            
            cP = cP + 1;
            
        end
    end
    X2 = P(1:(2*cP),1);
    Y2 = P(1:(2*cP),2);
    pgon = polyshape(X2,Y2);
    pgon = pgon.rmholes;
    X2 = [pgon.Vertices(:,1);pgon.Vertices(1,1)];
    Y2 = [pgon.Vertices(:,2);pgon.Vertices(1,2)];
end