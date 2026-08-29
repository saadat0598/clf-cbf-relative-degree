%% plot_pioneer_results.m
%


%% ---- Geometry & criteria (MUST match the controller's `obstacles`) ----
% [x_obs, y_obs, r_physical, r_safe]
obstacles = [ 3.6, 3.5, 0.30, 0.90 ;   % obstacle 1
              0.3, 6.5, 0.30, 0.90 ]; % obstacle 2
posTol = 0.15;          % [m] final position tolerance for "goal reached"


epsilonPt   = 0;         % m^2 -- point-CBF buffer (must match controller -- 0 since epsilon was removed)
epsilonBody = 0;         % m^2 -- body-CBF buffer  (must match controller -- 0 since epsilon was removed)
robotRadius = 0.30;      % m   -- must match controller
bodyMargin  = 0.05;      % m   -- must match controller
a_lookahead = 0.30;      % m   -- lookahead offset, must match controller's `a`

%% ---- Locate `out` and trajMatrix ----
if ~evalin('base','exist(''out'',''var'')')
    error('Variable ''out'' not found. Run the simulation first (Single simulation output must be checked).');
end
out = evalin('base','out');
if ~evalin('base','exist(''trajMatrix'',''var'')')
    error('trajMatrix not found. Run the trajectory generator first.');
end
trajMatrix = evalin('base','trajMatrix');
goal = trajMatrix(end,2:3);

%% ---- Pull logs (Structure With Time) ----
[t_pose, pose_v] = getLog(out, {'pose_log','pose','Pose'});
[t_v,    v_v]    = getLog(out, {'v_log','vCmd'});
[t_w,    w_v]    = getLog(out, {'w_log','yawRateCmd'});
[t_V,    V_v]    = getLog(out, {'Vout_log','V_out','V','Vx'});
[t_h,    h_v]    = getLog(out, {'h_log','h_out','h'});
[t_ex,   ex_v]   = getLog(out, {'ex_log','e_x'});
[t_ey,   ey_v]   = getLog(out, {'ey_log','e_y'});
[t_eth,  eth_v]  = getLog(out, {'eth_log','e_theta'});
[t_qp,   qp_v]   = getLog(out, {'qp_log','qp_status'});

%% ---- Fallback reconstructions (no extra Scopes needed) ----
% Pose from errors: x = x_d(t) + e_x, y = y_d(t) + e_y, th = th_d(t)+e_th
poseReconstructed = false;
if isempty(pose_v) && ~isempty(ex_v) && ~isempty(ey_v)
    tr  = t_ex;
    xd  = interp1(trajMatrix(:,1), trajMatrix(:,2), min(tr,trajMatrix(end,1)));
    yd  = interp1(trajMatrix(:,1), trajMatrix(:,3), min(tr,trajMatrix(end,1)));
    thd = interp1(trajMatrix(:,1), trajMatrix(:,4), min(tr,trajMatrix(end,1)));
    xR  = xd + ex_v(:,1);
    yR  = yd + ey_v(:,1);
    if ~isempty(eth_v), thR = thd + eth_v(:,1); else, thR = thd; end
    pose_v = [xR, yR, thR];  t_pose = tr;
    poseReconstructed = true;
    fprintf('[i] pose_log not logged -- pose RECONSTRUCTED from e_x/e_y + trajMatrix.\n');
end
% CLF value from reconstructed pose (offset-point definition, matches
% the controller's lookahead `a`)
if isempty(V_v) && ~isempty(pose_v)
    tr   = t_pose;
    xd  = interp1(trajMatrix(:,1), trajMatrix(:,2), min(tr,trajMatrix(end,1)));
    yd  = interp1(trajMatrix(:,1), trajMatrix(:,3), min(tr,trajMatrix(end,1)));
    thd = interp1(trajMatrix(:,1), trajMatrix(:,4), min(tr,trajMatrix(end,1)));
    pxq = pose_v(:,1) + a_lookahead*cos(pose_v(:,3));
    pyq = pose_v(:,2) + a_lookahead*sin(pose_v(:,3));
    pdx = xd + a_lookahead*cos(thd);
    pdy = yd + a_lookahead*sin(thd);
    V_v = (pxq-pdx).^2 + (pyq-pdy).^2;  t_V = tr;
    fprintf('[i] V not logged -- RECONSTRUCTED from pose + trajMatrix.\n');
end

%% ---- Figure 1: bird's-eye trajectory vs reference & both obstacles ----
figure('Name','Pioneer 3DX - Trajectory','Color','w');
hold on; axis equal; grid on;
theta_c = linspace(0,2*pi,100);

plot(trajMatrix(:,2), trajMatrix(:,3), 'k--', 'LineWidth',1.2, ...
     'DisplayName','Reference trajectory');

obsColors = {[0.85 0.3 0.3], [0.3 0.4 0.85]};   % obstacle 1 = red, obstacle 2 = blue
nObs = size(obstacles,1);
for k = 1:nObs
    xo = obstacles(k,1); yo = obstacles(k,2);
    rPhys = obstacles(k,3); rSafe = obstacles(k,4);
    fill(xo+rPhys*cos(theta_c), yo+rPhys*sin(theta_c), obsColors{k}, ...
         'EdgeColor','none','FaceAlpha',0.6, ...
         'DisplayName', sprintf('Obstacle %d', k));
    plot(xo+rSafe*cos(theta_c), yo+rSafe*sin(theta_c), '--', ...
         'Color', obsColors{k}, ...
         'DisplayName', sprintf('Safety boundary %d (r_{safe})', k));
end

if ~isempty(pose_v)
    x = pose_v(:,1); y = pose_v(:,2);
    plot(x, y, 'b-', 'LineWidth', 1.8, 'DisplayName','Robot path');
    plot(x(1), y(1), 'g^', 'MarkerFaceColor','g', 'MarkerSize',9, 'DisplayName','Start');
    plot(goal(1), goal(2), 'kp', 'MarkerFaceColor','y', 'MarkerSize',14, 'DisplayName','Goal');
    plot(x(end), y(end), 'bs', 'MarkerFaceColor','b', 'MarkerSize',8, 'DisplayName','Final position');

    % closest approach to EACH obstacle, marked separately
    dMinAll = zeros(nObs,1);
    approachMarkers = {'mo','co'};
    for k = 1:nObs
        dObs = hypot(x-obstacles(k,1), y-obstacles(k,2));
        [dMinAll(k), iM] = min(dObs);
        plot(x(iM), y(iM), approachMarkers{k}, 'MarkerFaceColor', approachMarkers{k}(1), ...
             'DisplayName', sprintf('Closest approach %d', k));
    end

    finalErr = hypot(x(end)-goal(1), y(end)-goal(2));
    title(sprintf('Trajectory%s  |  final pos err %.3f m  |  min dist obs1 %.3f m  |  min dist obs2 %.3f m', ...
          ternaryStr(poseReconstructed,' (pose reconstructed)',''), ...
          finalErr, dMinAll(1), dMinAll(2)));
else
    warning('pose_log not found/logged -- trajectory plot skipped.');
    title('Trajectory (pose\_log missing)');
end
xlabel('x [m]'); ylabel('y [m]');
legend('Location','bestoutside');

%% ---- Figure 2: diagnostics over time ----
figure('Name','Pioneer 3DX - Diagnostics','Color','w');

subplot(3,2,1);
plotIfPresent(t_v, v_v, 'v_{cmd} [m/s]', 'Linear velocity command');
hold on; yline(0.5,'k--'); yline(0,'k:');

subplot(3,2,2);
plotIfPresent(t_w, w_v, '\omega_{cmd} [rad/s]', 'Yaw rate command');
hold on; yline(1.5,'k--'); yline(-1.5,'k--'); yline(0,'k:');

subplot(3,2,3);
hold on;
plotIfPresent(t_ex, ex_v, 'error [m]', 'Position tracking errors');
plotIfPresent(t_ey, ey_v, 'error [m]', '');
legend({'e_x','e_y'},'Location','best');

subplot(3,2,4);
plotIfPresent(t_eth, eth_v, 'e_\theta [rad]', 'Heading tracking error');
hold on; yline(0,'k:');

subplot(3,2,5);
plotIfPresent(t_h, h_v, 'h(x) = min_k h_k(x)', 'Worst-case TIGHTENED CBF (point+body) -- must stay \geq 0');
hold on; yline(0,'r--','LineWidth',1.5);

subplot(3,2,6);

plotIfPresent(t_qp, qp_v, 'qp\_status', 'Controller status (0=nominal,1=safety QP active,3=INFEASIBLE,4=clamp)');
hold on; yline(0,'k:'); yline(1,'g:'); yline(3,'r-','LineWidth',1.2); ylim([-0.5 4.5]);

sgtitle('Pioneer 3DX - CLF-CBF-QP Controller Diagnostics (2 obstacles)');

%% ---- Figure 3: CLF value ----
figure('Name','Pioneer 3DX - CLF','Color','w');
plotIfPresent(t_V, V_v, 'V(x)', ...
    'CLF -- decays except while CBF overrides the unsafe reference');

%% ---- Lookahead point pt = pose + a*[cos th; sin th] ----
% MUST match the controller's offset exactly -- this is the point the
% point-CBF constraint is actually enforced on, NOT the raw pose.
ptx = []; pty = [];
if ~isempty(pose_v)
    ptx = pose_v(:,1) + a_lookahead*cos(pose_v(:,3));
    pty = pose_v(:,2) + a_lookahead*sin(pose_v(:,3));
end

%% ---- Figure 4: per-obstacle CBF margin over time -- TIGHTENED, both channels ----
if ~isempty(pose_v)
    figure('Name','Pioneer 3DX - Per-Obstacle CBF (tightened)','Color','w');
    hold on; grid on;
    obsLineColors = {[0.85 0.3 0.3], [0.3 0.4 0.85]};
    for k = 1:nObs
        % point-CBF (tightened, matches controller exactly)
        dvec2_pt = (ptx-obstacles(k,1)).^2 + (pty-obstacles(k,2)).^2;
        hk_pt_tilde = dvec2_pt - obstacles(k,4)^2 - epsilonPt;
        plot(t_pose, hk_pt_tilde, '-', 'LineWidth', 1.6, 'Color', obsLineColors{k}, ...
             'DisplayName', sprintf('h~_%d point-CBF (obstacle %d)', k, k));

        % body-CBF (tightened, matches controller exactly)
        r_body_safe = obstacles(k,3) + robotRadius + bodyMargin;
        dvec2_body = (pose_v(:,1)-obstacles(k,1)).^2 + (pose_v(:,2)-obstacles(k,2)).^2;
        hk_body_tilde = dvec2_body - r_body_safe^2 - epsilonBody;
        plot(t_pose, hk_body_tilde, '--', 'LineWidth', 1.2, 'Color', obsLineColors{k}*0.6+0.4, ...
             'DisplayName', sprintf('h~_%d body-CBF (obstacle %d)', k, k));
    end
    yline(0, 'k--', 'LineWidth', 1.2, 'DisplayName','safety boundary (tightened)');
    xlabel('t [s]'); ylabel('h~_k(x)');
    title('Per-obstacle TIGHTENED CBF: point-CBF (solid) vs. body-CBF (dashed) -- both must stay \geq 0');
    legend('Location','best');
end

%% ---- Goal assessment summary ----
fprintf('\n========== GOAL ASSESSMENT ==========\n');
if ~isempty(pose_v)
    finalErr = hypot(pose_v(end,1)-goal(1), pose_v(end,2)-goal(2));
    crit(finalErr < posTol, sprintf('Goal reached: final position error = %.3f m (tol %.2f m)', finalErr, posTol));
    
    for k = 1:nObs
        dObs = hypot(pose_v(:,1)-obstacles(k,1), pose_v(:,2)-obstacles(k,2));
        dMinK = min(dObs);
        crit(dMinK > obstacles(k,3), ...
             sprintf('No literal collision with obstacle %d: min distance = %.3f m (obstacle r = %.2f m)', ...
                     k, dMinK, obstacles(k,3)));
    end
end
if ~isempty(pose_v)
    fprintf('  [i]  --- The checks below use the SAME TIGHTENED margins the controller\n');
    fprintf('  [i]      enforces internally (h_tilde = h - epsilon, both point- and\n');
    fprintf('  [i]      body-CBF channels), so PASS here means PASS against the\n');
    fprintf('  [i]      controller''s actual requirement, not a looser proxy. ---\n');
    for k = 1:nObs
        % point-CBF, tightened
        dvec2_pt = (ptx-obstacles(k,1)).^2 + (pty-obstacles(k,2)).^2;
        hk_pt_tilde = dvec2_pt - obstacles(k,4)^2 - epsilonPt;
        crit(min(hk_pt_tilde) >= 0, ...
             sprintf('Point-CBF safety certificate, obstacle %d: min h~ = %.3f (must be >= 0)', k, min(hk_pt_tilde)));

        % body-CBF, tightened
        r_body_safe = obstacles(k,3) + robotRadius + bodyMargin;
        dvec2_body = (pose_v(:,1)-obstacles(k,1)).^2 + (pose_v(:,2)-obstacles(k,2)).^2;
        hk_body_tilde = dvec2_body - r_body_safe^2 - epsilonBody;
        crit(min(hk_body_tilde) >= 0, ...
             sprintf('Body-CBF safety certificate, obstacle %d: min h~ = %.3f (must be >= 0)', k, min(hk_body_tilde)));
    end
end
if ~isempty(h_v)
    hMin = min(h_v(:,1));
    crit(hMin >= 0, sprintf('Safety certificate (logged worst-case h_out, tightened): min h = %.3f (must be >= 0)', hMin));
end
if ~isempty(qp_v)
    nInfeasible = nnz(qp_v(:,1) == 3);
    crit(nInfeasible == 0, sprintf('No point-CBF infeasibility events: %d step(s) with qp_status == 3', nInfeasible));
    % Status 4 = defensive actuator clamp fired after the body-CBF cap.
    % Should be rare/never with the corrected controller (bounds are
    % already inside the joint QP); flagged as a diagnostic, not
    % necessarily a hard failure.
    nClamp = nnz(qp_v(:,1) == 4);
    crit(nClamp == 0, sprintf('No post-hoc actuator clamp events: %d step(s) with qp_status == 4', nClamp));
end
if ~isempty(ex_v) && ~isempty(ey_v)
    eNorm = hypot(ex_v(:,1), ey_v(:,1));
    fprintf('  [i]  Peak |e_p| (CBF override transient): %.3f m\n', max(eNorm));
    fprintf('  [i]  RMS position error over run: %.3f m\n', sqrt(mean(eNorm.^2)));
end
if ~isempty(eth_v)
    fprintf('  [i]  Final heading error: %.1f deg\n', rad2deg(eth_v(end,1)));
end
fprintf('=====================================\n\n');

%% ---------------- helper functions ----------------
function plotIfPresent(t, v, ylab, ttl)
    if isempty(t) || isempty(v)
        text(0.5,0.5,'(signal not logged)','HorizontalAlignment','center');
        title(ttl);
        return;
    end
    plot(t, v, 'LineWidth', 1.3);
    xlabel('t [s]'); ylabel(ylab); title(ttl); grid on;
end

function [t, v] = getLog(out, names)
    % names: char or cell array of candidate variable names (aliases)
    if ischar(names) || isstring(names), names = {char(names)}; end
    t = []; v = [];
    avail = out.who;
    for k = 1:numel(names)
        if any(strcmp(avail, names{k}))
            s = out.(names{k});
            t = s.time;
            v = squeeze(s.signals.values);
            if size(v,1) ~= numel(t), v = v.'; end   % rows = time
            return;
        end
    end
    warning('None of {%s} found on out -- skipping. Available: %s', ...
            strjoin(names,', '), strjoin(avail,', '));
end

function crit(ok, msg)
    if ok, fprintf('  [PASS] %s\n', msg);
    else,  fprintf('  [FAIL] %s\n', msg);
    end
end

function s = ternaryStr(c, a, b)
    if c, s = a; else, s = b; end
end