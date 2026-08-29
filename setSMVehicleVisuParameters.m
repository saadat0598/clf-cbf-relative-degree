                                                
% Sets parameters for Simscape Multibody Vehicle Visualizer
% Copyright 2017 The MathWorks, Inc.

smvv.cameras = struct;
smvv.cameras.frontView_offset = [-2 0 8];
smvv.cameras.sideView_offset = [-2 -7 -5];
smvv.cameras.driversView_offset = [0 0.25 0.7];

smvv.color = struct;
smvv.color.orange = [1 0.6 0];
smvv.color.red = [1 0.2 0];
smvv.color.dgreen = [0 0.4 0];
smvv.color.green = [0 0.6 0];
smvv.color.lgreen = [0.8 0.8 0];
smvv.color.dblue = [0 0.2 1];
smvv.color.blue = [0.2 0.6 1];
smvv.color.lblue = [0.4 0.8 1];
smvv.color.brown = [0.4 0.2 0];
smvv.color.lbrown = [0.8 0.4 0];
smvv.color.yellow = [1 0.8 0];
smvv.color.black = [0 0 0];
smvv.color.white = [1 1 1];
smvv.color.dgrey = [0.3 0.3 0.3];
smvv.color.grey = [0.5 0.5 0.5];
smvv.color.lgrey = [0.7 0.7 0.7];
smvv.color.vlgrey = [0.9 0.9 0.9];

smvv.filter = struct;
smvv.filter.t_c = 0.01;

smvv.finishline = struct;
smvv.finishline.length = 1;
smvv.finishline.width = 7.2;
smvv.finishline.depth = 0.005;
smvv.finishline.dim = [1 7.2 0.005];
smvv.finishline.offset = 259;
smvv.finishline.color = [1 0.2 0];
smvv.finishline.cones = struct;
smvv.finishline.cones.numCones = 7;
smvv.finishline.cones.offset = 278.7;
smvv.finishline.cones.spacing = 1.1;

smvv.road = struct;
smvv.road.markings = struct;
smvv.road.markings.segmentLength = 3;
smvv.road.markings.segmentWidth = 0.15;
smvv.road.markings.segmentDepth = 0.005;
smvv.road.markings.spacing = 9;
smvv.road.markings.numMarkings = 24;
smvv.road.markings.color = [1 1 1];
smvv.road.markings.extrusion = ...
  [0 0;
   279 0;
   279 0.10500000000000001;
   276 0.10500000000000001;
   276 0.05;
   267 0.05;
   267 0.10500000000000001;
   264 0.10500000000000001;
   264 0.05;
   255 0.05;
   255 0.10500000000000001;
   252 0.10500000000000001;
   252 0.05;
   243 0.05;
   243 0.10500000000000001;
   240 0.10500000000000001;
   240 0.05;
   231 0.05;
   231 0.10500000000000001;
   228 0.10500000000000001;
   228 0.05;
   219 0.05;
   219 0.10500000000000001;
   216 0.10500000000000001;
   216 0.05;
   207 0.05;
   207 0.10500000000000001;
   204 0.10500000000000001;
   204 0.05;
   195 0.05;
   195 0.10500000000000001;
   192 0.10500000000000001;
   192 0.05;
   183 0.05;
   183 0.10500000000000001;
   180 0.10500000000000001;
   180 0.05;
   171 0.05;
   171 0.10500000000000001;
   168 0.10500000000000001;
   168 0.05;
   159 0.05;
   159 0.10500000000000001;
   156 0.10500000000000001;
   156 0.05;
   147 0.05;
   147 0.10500000000000001;
   144 0.10500000000000001;
   144 0.05;
   135 0.05;
   135 0.10500000000000001;
   132 0.10500000000000001;
   132 0.05;
   123 0.05;
   123 0.10500000000000001;
   120 0.10500000000000001;
   120 0.05;
   111 0.05;
   111 0.10500000000000001;
   108 0.10500000000000001;
   108 0.05;
   99 0.05;
   99 0.10500000000000001;
   96 0.10500000000000001;
   96 0.05;
   87 0.05;
   87 0.10500000000000001;
   84 0.10500000000000001;
   84 0.05;
   75 0.05;
   75 0.10500000000000001;
   72 0.10500000000000001;
   72 0.05;
   63 0.05;
   63 0.10500000000000001;
   60 0.10500000000000001;
   60 0.05;
   51 0.05;
   51 0.10500000000000001;
   48 0.10500000000000001;
   48 0.05;
   39 0.05;
   39 0.10500000000000001;
   36 0.10500000000000001;
   36 0.05;
   27 0.05;
   27 0.10500000000000001;
   24 0.10500000000000001;
   24 0.05;
   15 0.05;
   15 0.10500000000000001;
   12 0.10500000000000001;
   12 0.05;
   3 0.05;
   3 0.10500000000000001;
   0 0.10500000000000001];
smvv.road.length = 279;
smvv.road.width = 7.2;
smvv.road.depth = 0.1;
smvv.road.dim = [279 7.2 0.1];
smvv.road.color = [0.5 0.5 0.5];

smvv.shortCone = struct;
smvv.shortCone.height = 0.5;
smvv.shortCone.baseThickness = 0.05;
smvv.shortCone.baseDiameter = 0.3;
smvv.shortCone.coneBaseDiameter = 0.2;
smvv.shortCone.coneTopDiameter = 0.05;
smvv.shortCone.reflectiveStripHeight = 0.25;
smvv.shortCone.coneColor = [1 0.6 0];
smvv.shortCone.reflectiveStripColor = [0.7 0.7 0.7];

smvv.slalom = struct;
smvv.slalom.gate = struct;
smvv.slalom.gate.cone_offset = 1.8;
smvv.slalom.startgate_offset = 57;
smvv.slalom.coneSpacing = 30;
smvv.slalom.numCones = 5;
smvv.slalom.firstConeFromStartgate = 26;
smvv.slalom.finishgateFromEndCone = 36;
smvv.slalom.firstCone_offset = 83;
smvv.slalom.endCone_offset = 203;
smvv.slalom.finishgate_offset = 239;

smvv.startline = struct;
smvv.startline.length = 1;
smvv.startline.width = 7.2;
smvv.startline.depth = 0.005;
smvv.startline.dim = [1 7.2 0.005];
smvv.startline.offset = 5;
smvv.startline.color = [0 0.6 0];

smvv.tallCone = struct;
smvv.tallCone.height = 1;
smvv.tallCone.baseThickness = 0.1;
smvv.tallCone.baseDiameter = 0.6;
smvv.tallCone.coneBaseDiameter = 0.4;
smvv.tallCone.coneTopDiameter = 0.1;
smvv.tallCone.reflectiveStripHeight = 0.5;
smvv.tallCone.coneColor = [1 0.6 0];
smvv.tallCone.reflectiveStripColor = [0.7 0.7 0.7];

smvv.vehicle = struct;
smvv.vehicle.wheelAssmRadius = 0.355;
smvv.vehicle.width = 1.97;
smvv.vehicle.frontAxle_offsetX = 1.51;
smvv.vehicle.rearAxle_offsetX = -1.49;
smvv.vehicle.wheel_offsetY = 0.81;
smvv.vehicle.axle_offsetZ = -0.26;
smvv.vehicle.ground_offsetZ = -0.603;
smvv.vehicle.body_offset = [2.75 0.985 -0.85];
smvv.vehicle.LR_offset = [-1.49 0.81 -0.26];
smvv.vehicle.RR_offset = [-1.49 -0.81 -0.26];
smvv.vehicle.LF_offset = [1.51 0.81 -0.26];
smvv.vehicle.RF_offset = [1.51 -0.81 -0.26];
smvv.vehicle.bodyColor = [0.2 0.6 1];
smvv.vehicle.posX_0 = 3.49;
smvv.vehicle.velX = struct;
smvv.vehicle.velX.time = [0; 2; 7; 21.534; 23.793999999999997; 25.793999999999997; ...
                     ];
smvv.vehicle.velX.signals = struct;
smvv.vehicle.velX.signals.values = [0; 0; 15; 15; 0; 0];
smvv.vehicle.posY = struct;
smvv.vehicle.posY.A = 1.47;

smvv.verge = struct;
smvv.verge.length = 279;
smvv.verge.width = 75;
smvv.verge.depth = 0.05;
smvv.verge.dim = [279 75 0.05];
smvv.verge.color = [0.8 1 0.8];

% Steering wheel
% long radius
swr1 = 0.15;   % long radius in meters
x1 = linspace(swr1,-swr1,20)';
y1 = sqrt(swr1.^2 - x1.^2);
x2 = linspace(-swr1,swr1,20)';
y2 = -sqrt(swr1.^2 - x2.^2);
xy1 = [x1,y1; x2(2:end-1),y2(2:end-1);x1(1),y1(1)];
% short radius
swr2 = 0.15-0.02;
x1 = linspace(swr2,-swr2,20)';
y1 = -sqrt(swr2.^2 - x1.^2);
x2 = linspace(-swr2,swr2,20)';
y2 = sqrt(swr2.^2 - x2.^2);
xy2 = [x1,y1; x2(2:end-1) y2(2:end-1);x1(1),y1(1)];
xy = [xy1;xy2];
smvv.steeringwheel.radius = swr1;
smvv.steeringwheel.extrusion = xy;
smvv.steeringwheel.thickness = 0.02;
smvv.steeringwheel.color = [0 0 0];
smvv.steeringwheel.honkRadius = 0.03;
smvv.steeringwheel.letterStick = 0.01;
smvv.steeringwheel.letterColor = [0 1 0];
clear swr1 swr2 xy xy1 xy2 x1 y1 x2 y2;


smvv.steeringwheel.ratio = 1;

% Later added to common data dictionary. 

