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


% Erosion (Contraction) of a convex polygon by a closed circle of radius r
%
% Input: 
%   X1, Y1  : Vertex Coordinates of a convex polygon
%   R       : Radius of the erosion (contraction) disk
% Output:
%   X2, Y2  : Coordinates of contracted convex polygon

function [X2, Y2] = cvxpolyerode(X1, Y1, R)
% Author: Omur Arslan, omur@seas.upenn.edu
% Date: August 03, 2015 Monday

epsilon = 1e-10;
X1 = X1(:);
Y1 = Y1(:);

if (length(X1) ~= length(Y1))
    error('Dimension Mismatch! Vertices should have the same number of coordinates.');
end
if (R < 0)
    error('Erosion (Contraction) radius should be nonnegative!');
end

N = length(X1); % Number of vertices
if (N < 3)
    % Check if the input polygon is trivial
    if (R > 0)
        X2 = [];
        Y2 = [];
    else 
        X2 = X1;
        Y2 = Y1;
    end
else
    % For nontrivial convex polygons: 
    % Determine if the vertices of the input polygon is in clockwise or
    % counter-clockwise order
    orientsign = 1 - 2 * ispolycw(X1, Y1);
    
    % Compute the contraction of input polygon
    P = [X1, Y1]; % Matrix representation of the input polygon
    Q = P; % Matrix representation of the output polygon
    for ck = 1:N
        cp = mod(ck-2,N) + 1; % Previous vertex index        
        ek = P(ck,:) - P(cp,:); % Edge vector        
        if (norm(ek) > epsilon)
            % if the current edge is nontrivial 
            
            nk = orientsign * [-ek(2); ek(1)]/norm(ek); % Edge Normal
            mk = P(ck,:) + R * nk'; % A point on the edge
            
            % Update Q by augmenting current edge
            M  = size(Q,1); 
            B = []; % Updated Cell Block
            for ci = 1: M
               cj = mod(ci, M) + 1; % Next vertex index
               % Vertex coordinates
               xi = Q(ci,:);
               xj = Q(cj,:);
               % Perpendicular signed distance to the augmented edge 
               si =  (xi - mk)*nk;  
               sj =  (xj - mk)*nk;
               % If vertices on the different side of the augmented edge,
               % include the crossing point
               if (si*sj) < 0
                   % Compute the point on the boundary and include it
                   w = ((mk - xj)*nk)/((xi - xj)*nk);
                   xx = w*xi + (1 - w)*xj;
                   B = [B; xx];
               end
               % If the next vertex on the correct side, include it in the
               % liest of vertices
               if (sj >= 0)
                   % Include this vertex
                   B = [B; xj];
               end
            end
            Q = B;
        end
    end
    % Coordinates of the Output Polygon
    if isempty(Q)
        X2 = [];
        Y2 = [];
    else
        X2 = Q(:,1);
        Y2 = Q(:,2);
    end
end