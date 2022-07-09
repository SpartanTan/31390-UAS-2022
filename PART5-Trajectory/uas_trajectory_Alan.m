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

% knots and corridor.times must be determined somehow!

% Trajectory generation
knots = [0 5 10 15];

waypoints = cell(1,4);
waypoints{1} = hc1;
waypoints{2} = hc2;
waypoints{3} = hc3;
waypoints{4} = hc4;

bc11 = hc1 - z1
bc12 = hc1 + z1

bc21 = hc2 - z1
bc22 = hc2 + z1

bc31 = hc3 - z1
bc32 = hc3 + z1

bc41 = hc4 - z1
bc42 = hc4 + z1

bsl = 1

b11 = [bc11-bsl/2, (bc11-bsl/2)+bsl]
b12 = [bc12-bsl/2, (bc12-bsl/2)+bsl]

b21 = [bc21-bsl/2, (bc21-bsl/2)+bsl]
b22 = [bc22-bsl/2, (bc22-bsl/2)+bsl]

b31 = [bc31-bsl/2, (bc31-bsl/2)+bsl]
b32 = [bc32-bsl/2, (bc32-bsl/2)+bsl]

b41 = [bc41-bsl/2, (bc41-bsl/2)+bsl]
b42 = [bc42-bsl/2, (bc42-bsl/2)+bsl]

order = 7;
corridors.times = [1 3 7 12];
corridors.x_lower = [b11(1,1) b12(1,1) b21(1,1) b22(1,1) b31(1,1) b32(1,1) b41(1,1) b42(1,1)];
corridors.x_upper = [b11(1,2) b12(1,2) b21(1,2) b22(1,2) b31(1,2) b32(1,2) b41(1,2) b42(1,2)];
corridors.y_lower = [b11(2,1) b12(2,1) b21(2,1) b22(2,1) b31(2,1) b32(2,1) b41(2,1) b42(2,1)];
corridors.y_upper = [b11(2,2) b12(2,2) b21(2,2) b22(2,2) b31(2,2) b32(2,2) b41(2,2) b42(2,2)];
corridors.z_lower = [b11(3,1) b12(3,1) b21(3,1) b22(3,1) b31(3,1) b32(3,1) b41(3,1) b42(3,1)];
corridors.z_upper = [b11(3,2) b12(3,2) b21(3,2) b22(3,2) b31(3,2) b32(3,2) b41(3,2) b42(3,2)];

make_plots = true;

poly_traj = uas_minimum_snap(knots, order, waypoints, corridors, make_plots);
