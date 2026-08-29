%% generate_trajMatrix.m
% Builds a smooth, dynamically feasible reference trajectory for the
% Pioneer 3DX model from the sparse waypoint list, in the same spirit as
% the parking model's trajMatrix (dense table, one row per sample).
%
%   trajMatrix = [t, x_d, y_d, theta_d, v_d, omega_d]      (N x 6)
%
% Construction:
%   1. Corners of the waypoint polyline are replaced by circular fillet
%      arcs of radius Rc  -> C1 path, bounded curvature (feasible for a
%      differential-drive robot).
%   2. A trapezoidal speed profile (accel aMax, cruise vCruise, decel to
%      zero at the end) parameterizes the path in time.
%   3. omega_d = v_d * kappa(s)  (zero on straights, v/Rc on arcs).
%
% Run this BEFORE simulating; it puts trajMatrix in the base workspace.
%
% NOTE: the reference trajectory below is generated purely from the
% waypoint polyline and does NOT itself route around obstacle 2 -- only
% the CLF-CBF-QP controller's CBF filter keeps the robot's actual path
% away from obstacle 2 at runtime. This plot is a visual sanity check to
% see how close the raw reference passes to each obstacle's safety
% boundary; if it cuts inside r_safe for a wide stretch, expect a longer
% CBF-driven detour there (as already happens for obstacle 1).

clear trajMatrix

% ---------------- User parameters ----------------
waypoints = [0 0; 4 2; 3 7; -3 6]

Rc      = 0.8;     % fillet radius (m). Must satisfy Rc >= vCruise/omegaMax
vCruise = 0.35;    % cruise speed (m/s)  (< vMax of the QP, for catch-up)
aMax    = 0.30;    % accel/decel (m/s^2)
dt      = 0.01;    % table sample time (s)
ds      = 0.005;   % path discretization (m)

% ---------------- Obstacle geometry (must match the controller) ----------
% [x_obs, y_obs, r_physical, r_safe]
obstacles = [ 3.6, 3.5, 0.30, 0.90 ;   % obstacle 1
              0.3, 6.5, 0.30, 0.90 ]; % obstacle 2

% ---------------- 1. Fillet the polyline ----------------
nW   = size(waypoints,1);
P    = {};                        % geometric primitives, in order
prevPt = waypoints(1,:);
for k = 2:nW-1
    A = waypoints(k-1,:); B = waypoints(k,:); C = waypoints(k+1,:);
    u1 = (B-A)/norm(B-A);  u2 = (C-B)/norm(C-B);
    cs = max(-1,min(1,u1*u2'));         % cos of turn angle
    Delta = acos(cs);                   % turn angle at this corner
    sgn   = sign(u1(1)*u2(2)-u1(2)*u2(1));   % +1 left, -1 right
    Tlen  = Rc*tan(Delta/2);            % tangent trim length
    % Guard: fillet must fit on both adjacent segments
    assert(Tlen < norm(B-A) && Tlen < norm(C-B), ...
        'Rc too large for corner %d — reduce Rc.', k);
    P1 = B - Tlen*u1;                   % arc entry point
    n1 = sgn*[-u1(2), u1(1)];           % normal toward turn center
    Ctr = P1 + Rc*n1;                   % arc center
    P{end+1} = struct('type','line','a',prevPt,'b',P1);            %#ok<SAGROW>
    a0 = atan2(P1(2)-Ctr(2), P1(1)-Ctr(1));
    P{end+1} = struct('type','arc','c',Ctr,'r',Rc, ...
                      'a0',a0,'da',sgn*Delta,'sgn',sgn);           %#ok<SAGROW>
    prevPt = B + Tlen*u2;               % arc exit point
end
P{end+1} = struct('type','line','a',prevPt,'b',waypoints(end,:));

% ---------------- 2. Sample path by arc length ----------------
xs = []; ys = []; th = []; ka = [];
for k = 1:numel(P)
    pk = P{k};
    if strcmp(pk.type,'line')
        Lseg = norm(pk.b-pk.a);
        m  = max(2, ceil(Lseg/ds));
        tt = linspace(0,1,m)';  if k>1, tt = tt(2:end); end
        xs = [xs; pk.a(1)+tt*(pk.b(1)-pk.a(1))];                   %#ok<AGROW>
        ys = [ys; pk.a(2)+tt*(pk.b(2)-pk.a(2))];                   %#ok<AGROW>
        hd = atan2(pk.b(2)-pk.a(2), pk.b(1)-pk.a(1));
        th = [th; hd*ones(numel(tt),1)];                           %#ok<AGROW>
        ka = [ka; zeros(numel(tt),1)];                             %#ok<AGROW>
    else
        Lseg = abs(pk.da)*pk.r;
        m  = max(2, ceil(Lseg/ds));
        aa = pk.a0 + linspace(0,pk.da,m)';  aa = aa(2:end);
        xs = [xs; pk.c(1)+pk.r*cos(aa)];                           %#ok<AGROW>
        ys = [ys; pk.c(2)+pk.r*sin(aa)];                           %#ok<AGROW>
        th = [th; aa + pk.sgn*pi/2];                               %#ok<AGROW>
        ka = [ka; (pk.sgn/pk.r)*ones(numel(aa),1)];                %#ok<AGROW>
    end
end
th   = unwrap(th);
s    = [0; cumsum(hypot(diff(xs),diff(ys)))];
Stot = s(end);

% ---------------- 3. Trapezoidal speed profile ----------------
sRamp = vCruise^2/(2*aMax);
if 2*sRamp > Stot                       % triangle profile fallback
    vPk = sqrt(aMax*Stot); tA = vPk/aMax; Ttot = 2*tA;
    sOfT = @(t) (t<=tA).*(0.5*aMax*t.^2) + ...
                (t> tA).*(Stot-0.5*aMax*max(Ttot-t,0).^2);
    vOfT = @(t) (t<=tA).*(aMax*t) + (t>tA).*(aMax*max(Ttot-t,0));
else
    tA = vCruise/aMax; sCr = Stot-2*sRamp; tCr = sCr/vCruise;
    Ttot = 2*tA + tCr;
    sOfT = @(t) (t<=tA).*(0.5*aMax*t.^2) + ...
                (t>tA & t<=tA+tCr).*(sRamp+vCruise*(t-tA)) + ...
                (t>tA+tCr).*(Stot-0.5*aMax*max(Ttot-t,0).^2);
    vOfT = @(t) (t<=tA).*(aMax*t) + (t>tA & t<=tA+tCr).*vCruise + ...
                (t>tA+tCr).*(aMax*max(Ttot-t,0));
end

% ---------------- 4. Assemble trajMatrix ----------------
tvec = (0:dt:Ttot)';
sq   = min(sOfT(tvec), Stot);
xd   = interp1(s, xs, sq);
yd   = interp1(s, ys, sq);
thd  = interp1(s, th, sq);
kq   = interp1(s, ka, sq);
vd   = vOfT(tvec);
wd   = vd.*kq;

trajMatrix = [tvec, xd, yd, thd, vd, wd];
assignin('base','trajMatrix',trajMatrix);

fprintf('trajMatrix: %d x 6, path length %.2f m, duration %.1f s\n', ...
        size(trajMatrix,1), Stot, Ttot);

% Report how close the RAW reference passes to each obstacle's safety
% boundary (informational only -- the CBF handles the actual avoidance).
for k = 1:size(obstacles,1)
    dObs = hypot(xs-obstacles(k,1), ys-obstacles(k,2));
    fprintf('  obstacle %d: closest raw-reference approach = %.3f m (r_safe = %.2f m)\n', ...
            k, min(dObs), obstacles(k,4));
end

%% ---------------- Quick visual check ----------------
figure; plot(xs,ys,'b','LineWidth',1.5); hold on; axis equal; grid on;
plot(waypoints(:,1),waypoints(:,2),'ko--');
phi = linspace(0,2*pi,100);

colors = {[0.85 0.2 0.2], [0.2 0.4 0.85]};   % obstacle 1 = red, obstacle 2 = blue
for k = 1:size(obstacles,1)
    xo = obstacles(k,1); yo = obstacles(k,2);
    rPhys = obstacles(k,3); rSafe = obstacles(k,4);
    plot(xo+rPhys*cos(phi), yo+rPhys*sin(phi), '-', 'Color', colors{k}, ...
         'LineWidth', 1.5, 'DisplayName', sprintf('Obstacle %d', k));
    plot(xo+rSafe*cos(phi), yo+rSafe*sin(phi), '--', 'Color', colors{k}, ...
         'DisplayName', sprintf('r_{safe} %d', k));
end
title('Reference trajectory vs obstacles'); xlabel('x (m)'); ylabel('y (m)');
legend('Location','bestoutside');