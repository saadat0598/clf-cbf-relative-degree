%% plot_parking_mpc.m
%
% Plotting and diagnostics for the two-loop controller:
%
%   OUTER: 5-Hz feedback-linearized MPC-CBF
%   INNER: 100-Hz feedback-linearization + discrete-CBF filter
%
% Required workspace variables before running:
%   out         -- Simulink simulation output
%   trajMatrix  -- 2000 x 6 reference:
%                  [t,x,y,theta,v,kappa]
%
% Recommended To Workspace logs, saved as "Structure With Time":
%
%   pose_log
%   v_log
%   delta_log
%   h_min_log
%   h_red_log
%   h_pink_log
%   h_obs3_log
%   qp_inner_log
%   qp_outer_log
%   scp_passes_log
%   u1_log
%   u2_log
%
% The script computes tracking errors ex, ey and eth from pose_log and
% trajMatrix; they no longer need to be logged separately.

clc;
close all;

%% ============================================================
%  Controller constants
% =============================================================

DT_outer = 0.20;
DT_inner = 0.01;
nSCP     = 3;

%% ============================================================
%  Geometry -- must match both controller scripts
% =============================================================

L   = 3.0;
ell = L/2;

x_obs1 = 55.0;
y_obs1 = 21.0;

x_obs2 = 40.0;
y_obs2 = 21.0;

x_obs3 = 26.6;
y_obs3 = 17.0;
r_obs3 = 1.2;

margin_base = 0.4;
k_v_margin  = 0.05;
v_park_nom  = 0.8;

r_margin = margin_base + k_v_margin*v_park_nom;   % 0.44 m
r_veh    = 1.6;
car_half_spacing = 1.5;

% Exact inner-CBF center separations.
r_pair_car = r_veh + r_veh + r_margin;            % 3.64 m
r_pair_3   = r_obs3 + r_veh + r_margin;           % 3.24 m

% Coarse outer-MPC obstacle circles.
r_outer_car = 4.20;
r_outer_3   = 3.30;

% Four obstacle circles representing the two parked cars.
x_c1r = x_obs1 - car_half_spacing;
y_c1r = y_obs1;

x_c1f = x_obs1 + car_half_spacing;
y_c1f = y_obs1;

x_c2r = x_obs2 - car_half_spacing;
y_c2r = y_obs2;

x_c2f = x_obs2 + car_half_spacing;
y_c2f = y_obs2;

x_goal = (x_obs1+x_obs2)/2 - L/2;
y_goal = y_obs1;

x_Sstart = 60.0;

vehicleColor = [0.20 0.60 1.00];
redCol       = [0.85 0.30 0.30];
pinkCol      = [0.90 0.50 0.80];
grayCol      = [0.55 0.55 0.55];
yellowClear  = [1.00 0.80 0.10];
frontCol     = [0.10 0.35 0.85];
refCol       = [0.10 0.70 0.10];

%% ============================================================
%  Retrieve simulation output and reference
% =============================================================

if ~evalin('base','exist(''out'',''var'')')
    error('Variable ''out'' was not found. Run the Simulink simulation first.');
end

out = evalin('base','out');

if ~evalin('base','exist(''trajMatrix'',''var'')')
    error(['Variable ''trajMatrix'' was not found. Run the trajectory ' ...
           'generator before running this plotting script.']);
end

trajMatrix = evalin('base','trajMatrix');

if size(trajMatrix,2) < 6
    error('trajMatrix must contain [t,x,y,theta,v,kappa].');
end

%% ============================================================
%  Retrieve logs
% =============================================================

[t_pose, pose_v] = getLogAny(out, {'pose_log','Pose_log','pose'});

[t_v, v_v] = getLogAny(out, ...
    {'v_log','v_out_log','v_out'});

[t_delta, delta_v] = getLogAny(out, ...
    {'delta_log','delta_out_log','delta_out'});

[t_hmin, hmin_v] = getLogAny(out, ...
    {'h_min_log','h_min_out_log','h_min_out'});

[t_hred, hred_v] = getLogAny(out, ...
    {'h_red_log','h_red_out_log','h_red_out'});

[t_hpink, hpink_v] = getLogAny(out, ...
    {'h_pink_log','h_pink_out_log','h_pink_out'});

[t_hobs3, hobs3_v] = getLogAny(out, ...
    {'h_obs3_log','h_obs3_out_log','h_obs3_out'});

[t_qp_inner, qp_inner_v] = getLogAny(out, ...
    {'qp_inner_log','qp_status_inner_log','qp_status_inner'});

[t_qp_outer, qp_outer_v] = getLogAny(out, ...
    {'qp_outer_log','qp_status_outer_log','qp_status_outer'});

[t_scp, scp_v] = getLogAny(out, ...
    {'scp_passes_log','scp_passes_out_log','scp_passes_out'});

[t_u1, u1_v] = getLogAny(out, ...
    {'u1_log','u1_cmd_log','u1_cmd'});

[t_u2, u2_v] = getLogAny(out, ...
    {'u2_log','u2_cmd_log','u2_cmd'});

%% ============================================================
%  Compute trajectory-tracking errors
% =============================================================
%
% ex  : along-track error in the reference heading frame
% ey  : cross-track error in the reference heading frame
% eth : wrapped heading error

ex_v  = [];
ey_v  = [];
eth_v = [];
t_err = [];

xref_v = [];
yref_v = [];
thref_v = [];
vref_v = [];

havePose = ~isempty(pose_v) && size(pose_v,2) >= 3;

if havePose
    nPose = size(pose_v,1);

    ex_v    = zeros(nPose,1);
    ey_v    = zeros(nPose,1);
    eth_v   = zeros(nPose,1);
    xref_v  = zeros(nPose,1);
    yref_v  = zeros(nPose,1);
    thref_v = zeros(nPose,1);
    vref_v  = zeros(nPose,1);

    t_err = t_pose;

    for i = 1:nPose
        [xr,yr,thr,vr,~] = interpolateReference( ...
            trajMatrix,t_pose(i));

        dx = pose_v(i,1) - xr;
        dy = pose_v(i,2) - yr;

        % Reference-heading frame.
        ex_v(i) =  cos(thr)*dx + sin(thr)*dy;
        ey_v(i) = -sin(thr)*dx + cos(thr)*dy;
        eth_v(i) = wrapToPi(pose_v(i,3) - thr);

        xref_v(i)  = xr;
        yref_v(i)  = yr;
        thref_v(i) = thr;
        vref_v(i)  = vr;
    end
end

%% ============================================================
%  Compute actual and reference clearances in metres
% =============================================================

redClear_actual   = [];
pinkClear_actual  = [];
obs3Clear_actual  = [];

redClear_ref = [];
pinkClearRef = [];
obs3ClearRef = [];

if havePose
    x  = pose_v(:,1);
    y  = pose_v(:,2);
    th = pose_v(:,3);

    xf = x + L*cos(th);
    yf = y + L*sin(th);

    redClear_actual = min(min( ...
        hypot(x-x_c1r,y-y_c1r), ...
        hypot(x-x_c1f,y-y_c1f)), ...
        min(hypot(xf-x_c1r,yf-y_c1r), ...
            hypot(xf-x_c1f,yf-y_c1f))) - r_pair_car;

    pinkClear_actual = min(min( ...
        hypot(x-x_c2r,y-y_c2r), ...
        hypot(x-x_c2f,y-y_c2f)), ...
        min(hypot(xf-x_c2r,yf-y_c2r), ...
            hypot(xf-x_c2f,yf-y_c2f))) - r_pair_car;

    obs3Clear_actual = min( ...
        hypot(x-x_obs3,y-y_obs3), ...
        hypot(xf-x_obs3,yf-y_obs3)) - r_pair_3;
end

xr_ref  = trajMatrix(:,2);
yr_ref  = trajMatrix(:,3);
thr_ref = trajMatrix(:,4);

xf_ref = xr_ref + L*cos(thr_ref);
yf_ref = yr_ref + L*sin(thr_ref);

redClear_ref = min(min( ...
    hypot(xr_ref-x_c1r,yr_ref-y_c1r), ...
    hypot(xr_ref-x_c1f,yr_ref-y_c1f)), ...
    min(hypot(xf_ref-x_c1r,yf_ref-y_c1r), ...
        hypot(xf_ref-x_c1f,yf_ref-y_c1f))) - r_pair_car;

pinkClearRef = min(min( ...
    hypot(xr_ref-x_c2r,yr_ref-y_c2r), ...
    hypot(xr_ref-x_c2f,yr_ref-y_c2f)), ...
    min(hypot(xf_ref-x_c2r,yf_ref-y_c2r), ...
        hypot(xf_ref-x_c2f,yf_ref-y_c2f))) - r_pair_car;

obs3ClearRef = min( ...
    hypot(xr_ref-x_obs3,yr_ref-y_obs3), ...
    hypot(xf_ref-x_obs3,yf_ref-y_obs3)) - r_pair_3;

%% ============================================================
%  Console summary
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' TWO-LOOP FEEDBACK-LINEARIZED MPC-CBF PARKING SUMMARY\n');
fprintf('============================================================\n');

printMinSignal('h_red  ',t_hred,hred_v);
printMinSignal('h_pink ',t_hpink,hpink_v);
printMinSignal('h_obs3 ',t_hobs3,hobs3_v);
printMinSignal('h_min  ',t_hmin,hmin_v);

fprintf('------------------------------------------------------------\n');

if ~isempty(obs3Clear_actual)
    [c3min,i3min] = min(obs3Clear_actual);
    fprintf('Actual min clearance to obstacle 3 : %+.3f m at t = %.2f s\n', ...
            c3min,t_pose(i3min));

    if c3min < 0
        fprintf('  *** OBSTACLE-3 SAFETY VIOLATION ***\n');
    end
end

if ~isempty(obs3ClearRef)
    fprintf('Reference min clearance to obstacle 3: %+.3f m\n', ...
            min(obs3ClearRef));

    if min(obs3ClearRef) < 0
        fprintf(['  WARNING: The reference itself violates the exact ' ...
                 'obstacle-3 CBF clearance.\n']);
        fprintf(['  A hard CBF controller cannot simultaneously follow ' ...
                 'this reference and remain safe.\n']);
    end
end

fprintf('Reference min clearance to RED       : %+.3f m\n', ...
        min(redClear_ref));
fprintf('Reference min clearance to PINK      : %+.3f m\n', ...
        min(pinkClearRef));

fprintf('------------------------------------------------------------\n');

if ~isempty(ex_v)
    fprintf('Final along-track error e_x          : %+.3f m\n',ex_v(end));
    fprintf('Final cross-track error e_y          : %+.3f m\n',ey_v(end));
    fprintf('Final heading error e_theta          : %+.3f rad (%+.2f deg)\n', ...
            eth_v(end),eth_v(end)*180/pi);
end

if havePose
    goalDist = hypot(pose_v(end,1)-x_goal, ...
                     pose_v(end,2)-y_goal);

    fprintf('Final Euclidean distance to goal     : %.3f m\n',goalDist);
end

fprintf('------------------------------------------------------------\n');

if ~isempty(qp_inner_v)
    fprintf('Inner QP faults, exitflag <= 0       : %d of %d\n', ...
            sum(qp_inner_v <= 0),numel(qp_inner_v));
else
    fprintf('Inner QP status                      : not logged\n');
end

if ~isempty(qp_outer_v)
    fprintf('Outer QP faults, exitflag <= 0       : %d of %d\n', ...
            sum(qp_outer_v <= 0),numel(qp_outer_v));
else
    fprintf('Outer QP status                      : not logged\n');
end

if ~isempty(scp_v)
    fprintf('Outer SCP passes                     : mean %.2f, max %d\n', ...
            mean(scp_v),max(scp_v));
else
    fprintf('Outer SCP passes                     : not logged\n');
end

fprintf('============================================================\n\n');

%% ============================================================
%  Figure 1: complete bird's-eye view
% =============================================================

figure('Name','Two-Loop MPC-CBF Parking - Bird''s-Eye View', ...
       'Color','w','Position',[80 80 1250 650]);

hold on;
axis equal;
grid on;

tc = linspace(0,2*pi,240);

% PINK parked car.
fill(x_c2r+r_veh*cos(tc), ...
     y_c2r+r_veh*sin(tc), ...
     pinkCol,'FaceAlpha',0.75, ...
     'EdgeColor',pinkCol*0.7, ...
     'LineWidth',1.2, ...
     'DisplayName','Pink car');

fill(x_c2f+r_veh*cos(tc), ...
     y_c2f+r_veh*sin(tc), ...
     pinkCol,'FaceAlpha',0.75, ...
     'EdgeColor',pinkCol*0.7, ...
     'LineWidth',1.2, ...
     'HandleVisibility','off');

plot(x_c2r+r_pair_car*cos(tc), ...
     y_c2r+r_pair_car*sin(tc), ...
     '--','Color',pinkCol, ...
     'LineWidth',1.1, ...
     'DisplayName','Pink exact CBF envelope');

plot(x_c2f+r_pair_car*cos(tc), ...
     y_c2f+r_pair_car*sin(tc), ...
     '--','Color',pinkCol, ...
     'LineWidth',1.1, ...
     'HandleVisibility','off');

plot(x_obs2+r_outer_car*cos(tc), ...
     y_obs2+r_outer_car*sin(tc), ...
     ':','Color',pinkCol*0.75, ...
     'LineWidth',1.8, ...
     'DisplayName','Pink outer-MPC circle');

% RED parked car.
fill(x_c1r+r_veh*cos(tc), ...
     y_c1r+r_veh*sin(tc), ...
     redCol,'FaceAlpha',0.75, ...
     'EdgeColor',redCol*0.7, ...
     'LineWidth',1.2, ...
     'DisplayName','Red car');

fill(x_c1f+r_veh*cos(tc), ...
     y_c1f+r_veh*sin(tc), ...
     redCol,'FaceAlpha',0.75, ...
     'EdgeColor',redCol*0.7, ...
     'LineWidth',1.2, ...
     'HandleVisibility','off');

plot(x_c1r+r_pair_car*cos(tc), ...
     y_c1r+r_pair_car*sin(tc), ...
     '--','Color',redCol, ...
     'LineWidth',1.1, ...
     'DisplayName','Red exact CBF envelope');

plot(x_c1f+r_pair_car*cos(tc), ...
     y_c1f+r_pair_car*sin(tc), ...
     '--','Color',redCol, ...
     'LineWidth',1.1, ...
     'HandleVisibility','off');

plot(x_obs1+r_outer_car*cos(tc), ...
     y_obs1+r_outer_car*sin(tc), ...
     ':','Color',redCol*0.75, ...
     'LineWidth',1.8, ...
     'DisplayName','Red outer-MPC circle');

% Obstacle 3.
fill(x_obs3+r_obs3*cos(tc), ...
     y_obs3+r_obs3*sin(tc), ...
     grayCol,'FaceAlpha',0.80, ...
     'EdgeColor',[0.25 0.25 0.25], ...
     'LineWidth',1.3, ...
     'DisplayName','Obstacle 3');

plot(x_obs3+r_pair_3*cos(tc), ...
     y_obs3+r_pair_3*sin(tc), ...
     '--','Color',[0.15 0.15 0.15], ...
     'LineWidth',1.2, ...
     'DisplayName','Obstacle-3 exact CBF envelope');

plot(x_obs3+r_outer_3*cos(tc), ...
     y_obs3+r_outer_3*sin(tc), ...
     ':','Color',[0.05 0.05 0.05], ...
     'LineWidth',1.8, ...
     'DisplayName','Obstacle-3 outer-MPC circle');

% Goal and reverse-S start.
plot(x_goal,y_goal, ...
     'kp','MarkerSize',13, ...
     'MarkerFaceColor',[0.95 0.90 0.10], ...
     'LineWidth',1.4, ...
     'DisplayName','Parking goal');

plot(x_Sstart,trajMatrix(1,3), ...
     'gs','MarkerSize',11, ...
     'MarkerFaceColor',[0.20 0.80 0.20], ...
     'DisplayName','Reverse-S start, x=60');

% Reference.
plot(trajMatrix(:,2),trajMatrix(:,3), ...
     '-','Color',refCol, ...
     'LineWidth',2.5, ...
     'DisplayName','Reference trajectory');

% Actual pose track.
if havePose
    plot(x,y, ...
         'k-','LineWidth',1.7, ...
         'DisplayName','Actual rear-axle track');

    plot(xf,yf, ...
         ':','Color',frontCol, ...
         'LineWidth',1.5, ...
         'DisplayName','Actual front-circle track');

    drawVehicleRectangle(x(1),y(1),th(1), ...
        [0.20 0.80 0.20],0.45,'Ego start');

    drawVehicleRectangle(x(end),y(end),th(end), ...
        yellowClear,0.65,'Ego end');

    % Ego two-circle footprint at the initial and final poses.
    for kk = [1,numel(x)]
        if kk == 1
            plot(x(kk)+r_veh*cos(tc), ...
                 y(kk)+r_veh*sin(tc), ...
                 '-.','Color',vehicleColor, ...
                 'LineWidth',1.4, ...
                 'DisplayName','Ego two-circle footprint');
        else
            plot(x(kk)+r_veh*cos(tc), ...
                 y(kk)+r_veh*sin(tc), ...
                 '-.','Color',vehicleColor, ...
                 'LineWidth',1.4, ...
                 'HandleVisibility','off');
        end

        plot(xf(kk)+r_veh*cos(tc), ...
             yf(kk)+r_veh*sin(tc), ...
             '-.','Color',vehicleColor, ...
             'LineWidth',1.4, ...
             'HandleVisibility','off');

        plot([x(kk) xf(kk)], ...
             [y(kk) yf(kk)], ...
             '-.','Color',vehicleColor, ...
             'LineWidth',1.0, ...
             'HandleVisibility','off');
    end
end

xlabel('x [m]');
ylabel('y [m]');
title('Two-loop MPC-CBF: actual track versus reference');
legend('Location','bestoutside');
xlim([-5 65]);
ylim([0 30]);

%% ============================================================
%  Figure 2: controller diagnostics
% =============================================================

figure('Name','Two-Loop MPC-CBF Parking - Diagnostics', ...
       'Color','w','Position',[120 60 1250 800]);

% ---------------- Commanded speed ----------------
subplot(4,2,1);
plotIfPresent(t_v,v_v, ...
    'v_{cmd} [m/s]', ...
    'Inner commanded speed');
hold on;
yline(0,'k:');

% ---------------- Steering ----------------
subplot(4,2,2);
plotIfPresent(t_delta,delta_v, ...
    '\delta [rad]', ...
    'Inner commanded steering');
hold on;
yline(0,'k:');
yline( 60*pi/180,'r--');
yline(-60*pi/180,'r--');
legend({'\delta','0','+\delta_{max}','-\delta_{max}'}, ...
       'Location','best');

% ---------------- ex / ey ----------------
subplot(4,2,3);
hold on;
grid on;

if ~isempty(ex_v)
    plot(t_err,ex_v,'LineWidth',1.3);
    plot(t_err,ey_v,'LineWidth',1.3);
    legend({'e_x along-track','e_y cross-track'}, ...
           'Location','best');
else
    text(0.5,0.5,'(pose log unavailable)', ...
         'HorizontalAlignment','center');
end

yline(0,'k:');
xlabel('t [s]');
ylabel('error [m]');
title('Computed trajectory-tracking position error');

% ---------------- Heading error ----------------
subplot(4,2,4);
plotIfPresent(t_err,eth_v, ...
    'e_\theta [rad]', ...
    'Computed heading-tracking error');
hold on;
yline(0,'k:');

% ---------------- Individual CBF values ----------------
subplot(4,2,5);
hold on;
grid on;

haveAnyH = false;

if ~isempty(hred_v)
    plot(t_hred,hred_v,'LineWidth',1.2);
    haveAnyH = true;
end

if ~isempty(hpink_v)
    plot(t_hpink,hpink_v,'LineWidth',1.2);
    haveAnyH = true;
end

if ~isempty(hobs3_v)
    plot(t_hobs3,hobs3_v,'LineWidth',1.2);
    haveAnyH = true;
end

if ~isempty(hmin_v)
    plot(t_hmin,hmin_v,'k','LineWidth',1.7);
    haveAnyH = true;
end

if haveAnyH
    legend({'h_{red}','h_{pink}','h_{obs3}','h_{min}'}, ...
           'Location','best');
else
    text(0.5,0.5,'(CBF logs unavailable)', ...
         'HorizontalAlignment','center');
end

yline(0,'r--','LineWidth',1.5);
xlabel('t [s]');
ylabel('h');
title('Inner discrete-CBF values -- all must remain \geq 0');

% ---------------- QP statuses ----------------
subplot(4,2,6);
hold on;
grid on;

haveQP = false;

if ~isempty(qp_inner_v)
    stairs(t_qp_inner,qp_inner_v,'LineWidth',1.4);
    haveQP = true;
end

if ~isempty(qp_outer_v)
    stairs(t_qp_outer,qp_outer_v,'LineWidth',1.4);
    haveQP = true;
end

if haveQP
    legend({'inner QP','outer QP'},'Location','best');
else
    text(0.5,0.5,'(QP logs unavailable)', ...
         'HorizontalAlignment','center');
end

yline(0,'r--');
yline(1,'g:');
xlabel('t [s]');
ylabel('exitflag');
title('QP statuses -- positive means solved');
ylim([-6 3]);

% ---------------- SCP passes ----------------
subplot(4,2,7);
plotIfPresent(t_scp,scp_v, ...
    'passes', ...
    'Outer SCP passes per MPC call');
hold on;
yline(nSCP,'r--','LineWidth',1.3);
ylim([0 nSCP+1]);

% ---------------- Outer virtual commands ----------------
subplot(4,2,8);
hold on;
grid on;

haveU = false;

if ~isempty(u1_v)
    stairs(t_u1,u1_v,'LineWidth',1.4);
    haveU = true;
end

if ~isempty(u2_v)
    stairs(t_u2,u2_v,'LineWidth',1.4);
    haveU = true;
end

if haveU
    legend({'u_1 = P_x velocity','u_2 = P_y velocity'}, ...
           'Location','best');
else
    text(0.5,0.5,'(outer command logs unavailable)', ...
         'HorizontalAlignment','center');
end

yline(0,'k:');
xlabel('t [s]');
ylabel('[m/s]');
title('Outer feedback-linearized virtual commands');

sgtitle('Two-Loop Feedback-Linearized MPC-CBF Diagnostics');

%% ============================================================
%  Figure 3: obstacle-3 avoidance zoom
% =============================================================

figure('Name','Obstacle-3 Avoidance Detail', ...
       'Color','w','Position',[180 120 1000 650]);

hold on;
axis equal;
grid on;

% Obstacle 3 and both safety envelopes.
fill(x_obs3+r_obs3*cos(tc), ...
     y_obs3+r_obs3*sin(tc), ...
     grayCol,'FaceAlpha',0.85, ...
     'EdgeColor',[0.20 0.20 0.20], ...
     'LineWidth',1.4, ...
     'DisplayName','Obstacle 3');

plot(x_obs3+r_pair_3*cos(tc), ...
     y_obs3+r_pair_3*sin(tc), ...
     '--','Color',[0.10 0.10 0.10], ...
     'LineWidth',1.3, ...
     'DisplayName','Exact inner-CBF boundary');

plot(x_obs3+r_outer_3*cos(tc), ...
     y_obs3+r_outer_3*sin(tc), ...
     ':','Color',[0.00 0.00 0.00], ...
     'LineWidth',2.0, ...
     'DisplayName','Conservative outer-MPC boundary');

% Reference and actual paths.
plot(trajMatrix(:,2),trajMatrix(:,3), ...
     '-','Color',refCol, ...
     'LineWidth',2.5, ...
     'DisplayName','Current reference');

if havePose
    plot(x,y, ...
         'k-','LineWidth',1.8, ...
         'DisplayName','Actual rear-axle track');

    plot(xf,yf, ...
         ':','Color',frontCol, ...
         'LineWidth',1.5, ...
         'DisplayName','Actual front-circle track');
end

% Exact inner-CBF center-clearance boundaries for obstacle 3.
yline(y_obs3-r_pair_3,'--', ...
      'Lower exact CBF boundary', ...
      'Color',[0.25 0.25 0.25]);

yline(y_obs3+r_pair_3,'--', ...
      'Upper exact CBF boundary', ...
      'Color',[0.25 0.25 0.25]);

% Conservative outer boundaries.
yline(y_obs3-r_outer_3,':', ...
      'Lower outer-MPC boundary', ...
      'Color',[0.00 0.00 0.00]);

yline(y_obs3+r_outer_3,':', ...
      'Upper outer-MPC boundary', ...
      'Color',[0.00 0.00 0.00]);

xlabel('x [m]');
ylabel('y [m]');
title('Obstacle-3 avoidance region');
legend('Location','bestoutside');

xlim([15 39]);
ylim([8 23]);

%% ============================================================
%  Figure 4: clearance in metres
% =============================================================

figure('Name','Actual Obstacle Clearances', ...
       'Color','w','Position',[180 100 1100 600]);

hold on;
grid on;

if ~isempty(redClear_actual)
    plot(t_pose,redClear_actual, ...
         'LineWidth',1.5, ...
         'DisplayName','RED clearance');
end

if ~isempty(pinkClear_actual)
    plot(t_pose,pinkClear_actual, ...
         'LineWidth',1.5, ...
         'DisplayName','PINK clearance');
end

if ~isempty(obs3Clear_actual)
    plot(t_pose,obs3Clear_actual, ...
         'LineWidth',2.0, ...
         'DisplayName','Obstacle-3 clearance');
end

if isempty(redClear_actual) && ...
   isempty(pinkClear_actual) && ...
   isempty(obs3Clear_actual)
    text(0.5,0.5, ...
         '(pose log unavailable)', ...
         'HorizontalAlignment','center');
end

yline(0,'r--','LineWidth',1.6, ...
      'DisplayName','Safety boundary');

xlabel('t [s]');
ylabel('clearance [m]');
title('Actual minimum centre clearance minus required CBF separation');
legend('Location','best');

%% ============================================================
%  Local plotting and data functions
% =============================================================

function plotIfPresent(t,v,ylab,ttl)

if isempty(t) || isempty(v)
    text(0.5,0.5, ...
         '(not logged)', ...
         'HorizontalAlignment','center');
    title(ttl);
    return;
end

plot(t,v,'LineWidth',1.3);
xlabel('t [s]');
ylabel(ylab);
title(ttl);
grid on;

end


function printMinSignal(name,t,v)

if isempty(t) || isempty(v)
    fprintf('%s                         : not logged\n',name);
    return;
end

[m,idx] = min(v(:));

fprintf('%s                         : %+.3f at t = %.2f s', ...
        name,m,t(idx));

if m < 0
    fprintf('   *** NEGATIVE ***');
end

fprintf('\n');

end


function drawVehicleRectangle(px,py,pth,col,alph,ename)

frontReach = 4.24;
rearReach  = 1.24;
halfWidth  = 0.985;

rectLocal = [ ...
     frontReach,  halfWidth;
     frontReach, -halfWidth;
    -rearReach,  -halfWidth;
    -rearReach,   halfWidth;
     frontReach,  halfWidth];

xr = px + rectLocal(:,1)*cos(pth) ...
        - rectLocal(:,2)*sin(pth);

yr = py + rectLocal(:,1)*sin(pth) ...
        + rectLocal(:,2)*cos(pth);

fill(xr,yr,col, ...
     'FaceAlpha',alph, ...
     'EdgeColor',col*0.6, ...
     'LineWidth',1.3, ...
     'DisplayName',ename);

end


function [xr,yr,thr,vr,kr] = interpolateReference(ref,tq)

n = size(ref,1);

% Locate the last row with increasing time. This ignores frozen-time
% padding at the end of trajMatrix.
nValid = 1;

for i = 2:n
    if ref(i,1) > ref(i-1,1) + 1e-9
        nValid = i;
    else
        break;
    end
end

if tq <= ref(1,1)
    xr  = ref(1,2);
    yr  = ref(1,3);
    thr = ref(1,4);
    vr  = ref(1,5);
    kr  = ref(1,6);
    return;
end

if tq >= ref(nValid,1)
    xr  = ref(nValid,2);
    yr  = ref(nValid,3);
    thr = ref(nValid,4);
    vr  = ref(nValid,5);
    kr  = ref(nValid,6);
    return;
end

idx = 1;

while idx < nValid-1 && ref(idx+1,1) < tq
    idx = idx + 1;
end

t0 = ref(idx,1);
t1 = ref(idx+1,1);

if t1 > t0
    alpha = (tq-t0)/(t1-t0);
else
    alpha = 0.0;
end

alpha = max(0.0,min(1.0,alpha));

xr  = (1-alpha)*ref(idx,2) + alpha*ref(idx+1,2);
yr  = (1-alpha)*ref(idx,3) + alpha*ref(idx+1,3);
thr = (1-alpha)*ref(idx,4) + alpha*ref(idx+1,4);
vr  = (1-alpha)*ref(idx,5) + alpha*ref(idx+1,5);
kr  = (1-alpha)*ref(idx,6) + alpha*ref(idx+1,6);

end


function a = wrapToPi(a)

a = atan2(sin(a),cos(a));

end


function [t,v] = getLogAny(out,names)
% Search SimulationOutput, logsout Dataset, and the base workspace.
% Preferred format is Structure With Time.

t = [];
v = [];

if ischar(names) || isstring(names)
    names = cellstr(names);
end

for k = 1:numel(names)

    name = names{k};
    s = [];
    found = false;

    % SimulationOutput field.
    try
        s = out.(name);
        found = true;
    catch
    end

    % Dataset logsout.
    if ~found
        try
            s = out.logsout.get(name);
            found = true;
        catch
        end
    end

    % Base-workspace variable from a To Workspace block.
    if ~found
        try
            if evalin('base',['exist(''' name ''',''var'')'])
                s = evalin('base',name);
                found = true;
            end
        catch
        end
    end

    if ~found
        continue;
    end

    % timeseries
    if isa(s,'timeseries')
        t = s.Time(:);
        v = s.Data;

    % Structure With Time
    elseif isstruct(s) && ...
           isfield(s,'time') && ...
           isfield(s,'signals')

        t = s.time(:);
        v = s.signals.values;

    else
        continue;
    end

    % Remove singleton dimensions.
    if ndims(v) > 2
        v = squeeze(v);
    end

    % Convert row data to column data.
    if isrow(v)
        v = v(:);
    end

    % A pose logged as 3 x 1 x N squeezes to 3 x N.
    if size(v,1) ~= numel(t) && ...
       size(v,2) == numel(t)
        v = v.';
    end

    if size(v,1) == numel(t)
        return;
    end
end

end