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

hc1 = [1.2681169509887695; -5.112864017486572; 1.3144266605377197]
hc2 = [-1.1366857290267944; -2.5462214946746826; 1.2406023740768433]
hc3 = [2.1060256958007812; -0.44249650835990906; 1.3313229084014893]
hc4 = [3.4874489307403564; -2.7299153804779053; 1.2378673553466797]
% quat1 = [-0.6704444289207458; -0.21129506826400757; 0.6283543109893799; 0.3332110047340393]
% quat2 = [-0.43534114956855774; 0.4569409489631653; 0.6442868709564209; 0.43194636702537537]
% quat3 = [-0.09150082617998123; 0.6443915367126465; -0.33262133598327637; 0.6824589371681213]
% quat4 = [0.7061073184013367; -0.06374791264533997; 0.11719270050525665; 0.6954240798950195]

quat1 = [0.3332110047340393; -0.6704444289207458; -0.21129506826400757; 0.6283543109893799]
quat2 = [0.43194636702537537; -0.43534114956855774; 0.4569409489631653; 0.6442868709564209]
quat3 = [0.6824589371681213; -0.09150082617998123; 0.6443915367126465; -0.33262133598327637; ]
quat4 = [0.6954240798950195; 0.7061073184013367; -0.06374791264533997; 0.11719270050525665; ]

rm1 = quat2rotm(quat1')
rm2 = quat2rotm(quat2')
rm3 = quat2rotm(quat3')
rm4 = quat2rotm(quat4')

% Trajectory generation
knots = [0 5 10 15 20 25 30 35];

d = [1;1;1]
scale = 0.1

bc11 = hc1 - (d+rm1(:,3))*scale
bc12 = hc1 + (d+rm1(:,3))*scale

bc21 = hc2 - (d+rm2(:,3))*scale
bc22 = hc2 + (d+rm2(:,3))*scale

bc31 = hc3 - (d+rm3(:,3))*scale
bc32 = hc3 + (d+rm3(:,3))*scale

bc41 = hc4 - (d+rm4(:,3))*scale
bc42 = hc4 + (d+rm4(:,3))*scale

waypoints = cell(1,8);
waypoints{1} = bc11;
waypoints{2} = bc12;
waypoints{3} = bc21;
waypoints{4} = bc22;
waypoints{5} = bc31;
waypoints{6} = bc32;
waypoints{7} = bc41;
waypoints{8} = bc42;

b1 = [bc11, bc12]

b2 = [bc21, bc22]

b3 = [bc31, bc32]

b4 = [bc41, bc42]

order = 9;
corridors.times = [3 12 23 32];
corridors.x_lower = [b1(1,1) b2(1,1) b3(1,1) b4(1,1)];
corridors.x_upper = [b1(1,2) b2(1,2) b3(1,2) b4(1,2)];
corridors.y_lower = [b1(2,1) b2(2,1) b3(2,1) b4(2,1)];
corridors.y_upper = [b1(2,2) b2(2,2) b3(2,2) b4(2,2)];
corridors.z_lower = [b1(3,1) b2(3,1) b3(3,1) b4(3,1)];
corridors.z_upper = [b1(3,2) b2(3,2) b3(3,2) b4(3,2)];

make_plots = true;

poly_traj = uas_minimum_snap(knots, order, waypoints, corridors, make_plots);
