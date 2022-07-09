%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% MIT License
% 
% Copyright (c) 2021 David Wuthier (dwuthier@gmail.com)
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
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
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialization
% close all
% clear
% clc


l1 = [1.49 ; -5.13; 1.27];
l2 = [-1.60; -2.73; 1.22];
l3 = [2.05; 0.93; 1.29];
l4 = [3.71; -2.64; 1.13];

home = [1.55; -2.64; 0.4];

% tolerance
t = 0.01;
% Trajectory generation
%uas_parameters;

knots = [0 19];
waypoints = cell(1,2);
waypoints{1} = home;
waypoints{2} = home;


order = 10;
corridors.times =   [2.5        4           6           10          14];
corridors.x_lower = [3 - 0.2    l1(1) - t   l2(1) - t   l3(1) - t   l4(1) - t];
corridors.x_upper = [3 + 0.2    l1(1) + t   l2(1) + t   l3(1) + t   l4(1) + t];
corridors.y_lower = [-4.5       l1(2) - t   l2(2) - t   l3(2) - t   l4(2) - t];
corridors.y_upper = [-4.5 + 0.5 l1(2) + t   l2(2) + t   l3(2) + t   l4(2) + t];
corridors.z_lower = [0.3        l1(3) - t   l2(3) + t   l3(3) - t   l4(3) - t];
corridors.z_upper = [1.2        l1(3) + t   l2(3) + t   l3(3) + t   l4(3) + t];

% Fix this...
% order = 7;
% corridors.times = [2];
% corridors.x_lower = [-0,36 4 9 8];
% corridors.x_upper = [-  5 10 10];
% corridors.y_lower = [-1 0 -1 8];
% corridors.y_upper = [1 1 1 10];
% corridors.z_lower = [0 0 0 0];
% corridors.z_upper = [2 2 2 2];
% ...until here
make_plots = true;

poly_traj = uas_minimum_snap(knots, order, waypoints, corridors, make_plots);
disp(poly_traj)