%% plot_nmpc_result.m
%
% Result plots for the fmincon-NMPC outer loop (mpc_outer_fmincon_nmpc)
% with the unchanged FL+CBF inner filter.
%
% Prerequisites: run the simulation, then call save_parking_run as usual
% (make sure the new solve_ms signal is logged too -- see below), e.g.:
%   run_NMPC_N5_g0p3.mat
%
% To log solve_ms: connect the outer block's 7th output (solve_ms) to a
% To Workspace block named solve_ms (Save format: Timeseries or
% Structure With Time), then add to save_parking_run.m:
%   solve_ms = ...;  t_solve = ...;   % same pattern as qpo/scp
%   save(fileName, ... ,'solve_ms','t_solve', ...);
% If solve_ms is missing from the file, the script skips Figure D
% gracefully.
%
% Produces:
%   Figure A  full bird's-eye view (NMPC vs optional SCP-QP baseline)
%   Figure B  zoom on the decision region (obstacle-3 exit + parking bay)
%   Figure C  barrier-value overlays h_red, h_pink, h_obs3, h_min [m^2]
%   Figure D  NMPC diagnostics: solve time, mode timeline, speed
%   Console   comparison table incl. solve-time statistics
%
% Styling matches plot_horizon_overlay / plot_gamma_overlay.

clc;
close all;

%% ============================================================
%  Runs: {label, file, colour}.  First row = NMPC run (required).
%  Add the SCP-QP baseline (or any other run) for overlay, or leave
%  only the NMPC row.
% =============================================================

runs = { ...
    'NMPC (fmincon), N_p = 5', 'run_NMPC_N5_g0p3.mat', [0.10 0.35 0.85]; ...
    'SCP-QP baseline, N_p = 5','run_N5_g0p3.mat',      [0.10 0.60 0.20]  };

savePNGs = true;      % export 300-dpi PNGs next to the .mat files

%% ============================================================
%  Geometry -- identical to plot_horizon_overlay / both controllers
% =============================================================

L   = 3.0;

x_obs1 = 55.0;  y_obs1 = 21.0;
x_obs2 = 39.0;  y_obs2 = 21.0;
x_obs3 = 26.6;  y_obs3 = 17.0;
r_obs3 = 1.2;

r_pair_car = 3.64;
r_pair_3   = 3.24;
r_outer_3  = 3.30;

r_veh = 1.6;
car_half_spacing = 1.5;

x_c1r = x_obs1 - car_half_spacing;  y_c1r = y_obs1;
x_c1f = x_obs1 + car_half_spacing;  y_c1f = y_obs1;
x_c2r = x_obs2 - car_half_spacing;  y_c2r = y_obs2;
x_c2f = x_obs2 + car_half_spacing;  y_c2f = y_obs2;

x_goal = (x_obs1+x_obs2)/2 - L/2 - 1;
y_goal = y_obs1;

gateRear1 = [21.0;13.0];
gateRear2 = [31.0;13.0];
yLane     = 16.36;
x_Sstart  = 60.0;

G = struct('x_obs1',x_obs1,'y_obs1',y_obs1,'x_obs2',x_obs2,'y_obs2',y_obs2, ...
           'x_obs3',x_obs3,'y_obs3',y_obs3,'r_obs3',r_obs3, ...
           'r_pair_car',r_pair_car,'r_pair_3',r_pair_3,'r_outer_3',r_outer_3, ...
           'r_veh',r_veh,'x_c1r',x_c1r,'y_c1r',y_c1r,'x_c1f',x_c1f,'y_c1f',y_c1f, ...
           'x_c2r',x_c2r,'y_c2r',y_c2r,'x_c2f',x_c2f,'y_c2f',y_c2f, ...
           'x_goal',x_goal,'y_goal',y_goal, ...
           'gateRear1',gateRear1,'gateRear2',gateRear2, ...
           'yLane',yLane,'x_Sstart',x_Sstart);

%% ============================================================
%  Load runs (missing optional rows are skipped)
% =============================================================

nKeep = 0;
R     = {};
trajM = [];

for k = 1:size(runs,1)
    file = runs{k,2};

    if exist(file,'file') ~= 2
        if k == 1
            error(['File ''%s'' not found. Re-run the NMPC simulation ' ...
                   'and call save_parking_run afterwards.'],file);
        else
            fprintf('Note: ''%s'' not found -- skipping overlay ''%s''.\n', ...
                    file,runs{k,1});
            continue;
        end
    end

    S = load(file);

    nKeep = nKeep+1;
    R{nKeep}.lab  = runs{k,1};
    R{nKeep}.file = file;
    R{nKeep}.col  = runs{k,3};
    R{nKeep}.pose = S.pose;
    R{nKeep}.t    = S.t_pose;
    R{nKeep}.v    = S.v;
    R{nKeep}.tv   = S.t_v;
    R{nKeep}.hred = S.hred;    R{nKeep}.thred  = S.t_hred;
    R{nKeep}.hpink= S.hpink;   R{nKeep}.thpink = S.t_hpink;
    R{nKeep}.hobs3= S.hobs3;   R{nKeep}.thobs3 = S.t_hobs3;
    R{nKeep}.hmin = S.hmin;    R{nKeep}.thmin  = S.t_hmin;
    R{nKeep}.mode = S.mode;    R{nKeep}.tmode  = S.t_mode;
    R{nKeep}.qpo  = S.qpo;     R{nKeep}.tqpo   = S.t_qpo;
    R{nKeep}.scp  = S.scp;     R{nKeep}.tscp   = S.t_scp;

    % New NMPC-only signals (optional)
    if isfield(S,'solve_ms')
        R{nKeep}.solve  = S.solve_ms;
        R{nKeep}.tsolve = S.t_solve;
    else
        R{nKeep}.solve  = [];
        R{nKeep}.tsolve = [];
    end

    if isempty(trajM)
        trajM = S.trajMatrix;
    end
end

nR = nKeep;

% Freeze time per run (same rule as plot_horizon_overlay).
for k = 1:nR
    R{k}.tFreeze = NaN;
    if ~isempty(R{k}.v)
        iMove = find(abs(R{k}.v) > 0.05,1,'last');
        if ~isempty(iMove) && iMove < numel(R{k}.v) && ...
                R{k}.tv(end) - R{k}.tv(iMove) > 2.0
            R{k}.tFreeze = R{k}.tv(min(iMove+1,numel(R{k}.tv)));
        end
    end
end

%% ============================================================
%  Figure A: full bird's-eye overlay
% =============================================================

figA = figure('Name','NMPC run - bird''s-eye view', ...
              'Color','w','Position',[60 80 1250 650]);
hold on; axis equal; grid on;

drawScenery(G,trajM);

for k = 1:nR
    x = R{k}.pose(:,1);
    y = R{k}.pose(:,2);

    plot(x,y,'-','Color',R{k}.col,'LineWidth',1.9, ...
         'DisplayName',R{k}.lab);

    if ~isnan(R{k}.tFreeze)
        [~,ip] = min(abs(R{k}.t - R{k}.tFreeze));
        plot(x(ip),y(ip),'x','MarkerSize',11,'LineWidth',2.2, ...
             'Color',R{k}.col,'HandleVisibility','off');
    end
end

xlabel('x [m]');
ylabel('y [m]');
title('Two-loop NMPC-CBF: closed-loop track (fmincon outer, FL+CBF inner)');
legend('Location','eastoutside');
xlim([-3 66]);
ylim([0 30]);

%% ============================================================
%  Figure B: decision-region zoom
% =============================================================

figB = figure('Name','NMPC run - decision-region zoom', ...
              'Color','w','Position',[100 60 1250 650]);
hold on; axis equal; grid on;

drawScenery(G,trajM);

for k = 1:nR
    x = R{k}.pose(:,1);
    y = R{k}.pose(:,2);

    plot(x,y,'-','Color',R{k}.col,'LineWidth',1.9, ...
         'DisplayName',R{k}.lab);

    if ~isnan(R{k}.tFreeze)
        [~,ip] = min(abs(R{k}.t - R{k}.tFreeze));
        plot(x(ip),y(ip),'x','MarkerSize',11,'LineWidth',2.2, ...
             'Color',R{k}.col,'HandleVisibility','off');
    end
end

xlabel('x [m]');
ylabel('y [m]');
title('Zoom: obstacle-3 exit and parking bay (''x'' marks where motion ceases)');
legend('Location','eastoutside');
xlim([23 63]);
ylim([10.5 24]);

%% ============================================================
%  Figure C: barrier-value overlays
% =============================================================

figC = figure('Name','NMPC run - barrier values', ...
              'Color','w','Position',[140 40 1150 780]);

specs = { 'hred', 'thred',  'h_{red}  [m^2]';
          'hpink','thpink', 'h_{pink} [m^2]';
          'hobs3','thobs3', 'h_{obs3} [m^2]';
          'hmin', 'thmin',  'h_{min}  [m^2]' };

yClip = 80;

for s = 1:4
    subplot(2,2,s);
    hold on; grid on;

    for k = 1:nR
        tv = R{k}.(specs{s,2});
        vv = R{k}.(specs{s,1});

        if isempty(tv)
            continue;
        end

        if s == 1
            plot(tv,vv,'-','Color',R{k}.col,'LineWidth',1.4, ...
                 'DisplayName',R{k}.lab);
        else
            plot(tv,vv,'-','Color',R{k}.col,'LineWidth',1.4, ...
                 'HandleVisibility','off');
        end
    end

    yline(0,'--','Color',[0.5 0.5 0.5],'LineWidth',1.2, ...
          'HandleVisibility','off');

    xlabel('t [s]');
    ylabel(specs{s,3});
    ylim([-2 yClip]);
    xlim([0 120]);

    if s == 1
        legend('Location','northeast');
        title('Barrier values per run (y-axis clipped; initial h exceeds 2000)');
    end
end

%% ============================================================
%  Figure D: NMPC diagnostics (solve time, mode, speed)
% =============================================================

haveSolve = ~isempty(R{1}.solve);

if haveSolve
    figD = figure('Name','NMPC diagnostics', ...
                  'Color','w','Position',[180 60 1150 780]);

    subplot(3,1,1);
    hold on; grid on;
    for k = 1:nR
        if isempty(R{k}.solve)
            continue;
        end
        plot(R{k}.tsolve,R{k}.solve,'-','Color',R{k}.col,'LineWidth',1.2, ...
             'DisplayName',R{k}.lab);
    end
    yline(200,'--','Color',[0.5 0.5 0.5],'LineWidth',1.2, ...
          'DisplayName','5 Hz budget (200 ms)');
    xlabel('t [s]');
    ylabel('solve time [ms]');
    title('fmincon (SQP) solve time per outer call');
    legend('Location','northeast');

    subplot(3,1,2);
    hold on; grid on;
    for k = 1:nR
        if isempty(R{k}.mode)
            continue;
        end
        stairs(R{k}.tmode,R{k}.mode,'-','Color',R{k}.col,'LineWidth',1.2, ...
               'DisplayName',R{k}.lab);
    end
    yticks(1:8);
    yticklabels({'TRACK','AVOID','REJOIN','STOP@S','REV-S','PARKED', ...
                 'ESCAPE','SAFE-HOLD'});
    ylim([0.5 8.5]);
    xlabel('t [s]');
    title('Mode timeline');
    legend('Location','southeast');

    subplot(3,1,3);
    hold on; grid on;
    for k = 1:nR
        if isempty(R{k}.v)
            continue;
        end
        plot(R{k}.tv,R{k}.v,'-','Color',R{k}.col,'LineWidth',1.2, ...
             'DisplayName',R{k}.lab);
    end
    yline(0,'--','Color',[0.5 0.5 0.5],'LineWidth',1.0, ...
          'HandleVisibility','off');
    xlabel('t [s]');
    ylabel('v [m/s]');
    title('Applied speed (inner filter output)');
    legend('Location','northeast');
else
    figD = [];
    fprintf(['Note: solve_ms not found in ''%s'' -- Figure D skipped. ' ...
             'Log the outer block''s solve_ms output to enable it.\n'], ...
            R{1}.file);
end

%% ============================================================
%  Console comparison table
% =============================================================

fprintf('\n');
fprintf('==================================================================\n');
fprintf(' NMPC RUN SUMMARY (fmincon outer + FL+CBF inner)\n');
fprintf('==================================================================\n');
fprintf(['  run                        | outcome        freeze t | goal dist | ' ...
         'e_theta   | min h_red | min h_pink | min h_obs3 | NLP ok  | solve ms (mean/med/max)\n']);
fprintf(['-----------------------------+---------------------------+-----------+' ...
         '----------+-----------+------------+------------+---------+------------------------\n']);

for k = 1:nR
    if ~isempty(R{k}.mode) && any(R{k}.mode == 6)
        ocStr = 'PARKED';
    elseif ~isempty(R{k}.mode)
        ocStr = sprintf('NOT (%s)',modeNameLocal(R{k}.mode(end)));
    else
        ocStr = 'no mode log';
    end

    if isnan(R{k}.tFreeze)
        frStr = '   -    ';
    else
        frStr = sprintf('%7.2f  ',R{k}.tFreeze);
    end

    xf_ = R{k}.pose(end,1);
    yf_ = R{k}.pose(end,2);
    thf_= R{k}.pose(end,3);

    goalDist = hypot(xf_-G.x_goal,yf_-G.y_goal);
    eth      = wrapToPiLocal(thf_ - trajM(end,4));

    [mh_r,~] = minNonempty(R{k}.hred);
    [mh_p,~] = minNonempty(R{k}.hpink);
    [mh_3,~] = minNonempty(R{k}.hobs3);

    % For the NMPC file, scp carries the 1/0 solve-success flag;
    % for the SCP baseline it carries the pass count. Report success
    % rate (NLP) or mean passes (SCP) accordingly.
    if ~isempty(R{k}.scp)
        if all(ismember(R{k}.scp,[0 1]))
            okStr = sprintf('%3d/%3d',sum(R{k}.scp==1),numel(R{k}.scp));
        else
            okStr = sprintf('SCP %.2f',mean(R{k}.scp));
        end
    else
        okStr = '   -   ';
    end

    if ~isempty(R{k}.solve)
        svStr = sprintf('%6.1f/%6.1f/%6.1f', ...
                mean(R{k}.solve),median(R{k}.solve),max(R{k}.solve));
    else
        svStr = '         -         ';
    end

    fprintf('  %-27s | %-14s %s | %8.3f  | %+7.2f dg | %+8.3f | %+10.3f | %+10.3f | %s | %s\n', ...
            R{k}.lab,ocStr,frStr,goalDist,eth*180/pi, ...
            mh_r,mh_p,mh_3,okStr,svStr);
end

fprintf('==================================================================\n\n');

%% ============================================================
%  Export
% =============================================================

if savePNGs
    try
        exportgraphics(figA,'nmpc_tracks_full.png','Resolution',300);
        exportgraphics(figB,'nmpc_tracks_zoom.png','Resolution',300);
        exportgraphics(figC,'nmpc_barriers.png','Resolution',300);
        msg = 'Exported nmpc_tracks_full.png, nmpc_tracks_zoom.png, nmpc_barriers.png';
        if ~isempty(figD)
            exportgraphics(figD,'nmpc_diagnostics.png','Resolution',300);
            msg = [msg ', nmpc_diagnostics.png'];
        end
        fprintf('%s\n',msg);
    catch ME
        warning('PNG export failed (%s). Figures are still open.',ME.message);
    end
end

%% ============================================================
%  Local functions
% =============================================================

function drawScenery(G,trajM)
% Static parking-lot scenery, styled like plot_parking_mpc Figure 1.

pinkCol = [0.90 0.50 0.80];
redCol  = [0.85 0.30 0.30];
grayCol = [0.55 0.55 0.55];
refCol  = [0.10 0.70 0.10];

tc = linspace(0,2*pi,240);

fill(G.x_c2r+G.r_veh*cos(tc),G.y_c2r+G.r_veh*sin(tc),pinkCol, ...
     'FaceAlpha',0.75,'EdgeColor',pinkCol*0.7,'LineWidth',1.2, ...
     'DisplayName','Pink car');
fill(G.x_c2f+G.r_veh*cos(tc),G.y_c2f+G.r_veh*sin(tc),pinkCol, ...
     'FaceAlpha',0.75,'EdgeColor',pinkCol*0.7,'LineWidth',1.2, ...
     'HandleVisibility','off');
plot(G.x_c2r+G.r_pair_car*cos(tc),G.y_c2r+G.r_pair_car*sin(tc),'--', ...
     'Color',pinkCol,'LineWidth',1.1, ...
     'DisplayName','Exact CBF envelopes');
plot(G.x_c2f+G.r_pair_car*cos(tc),G.y_c2f+G.r_pair_car*sin(tc),'--', ...
     'Color',pinkCol,'LineWidth',1.1,'HandleVisibility','off');

fill(G.x_c1r+G.r_veh*cos(tc),G.y_c1r+G.r_veh*sin(tc),redCol, ...
     'FaceAlpha',0.75,'EdgeColor',redCol*0.7,'LineWidth',1.2, ...
     'DisplayName','Red car');
fill(G.x_c1f+G.r_veh*cos(tc),G.y_c1f+G.r_veh*sin(tc),redCol, ...
     'FaceAlpha',0.75,'EdgeColor',redCol*0.7,'LineWidth',1.2, ...
     'HandleVisibility','off');
plot(G.x_c1r+G.r_pair_car*cos(tc),G.y_c1r+G.r_pair_car*sin(tc),'--', ...
     'Color',redCol,'LineWidth',1.1,'HandleVisibility','off');
plot(G.x_c1f+G.r_pair_car*cos(tc),G.y_c1f+G.r_pair_car*sin(tc),'--', ...
     'Color',redCol,'LineWidth',1.1,'HandleVisibility','off');

fill(G.x_obs3+G.r_obs3*cos(tc),G.y_obs3+G.r_obs3*sin(tc),grayCol, ...
     'FaceAlpha',0.80,'EdgeColor',[0.25 0.25 0.25],'LineWidth',1.3, ...
     'DisplayName','Obstacle 3');
plot(G.x_obs3+G.r_pair_3*cos(tc),G.y_obs3+G.r_pair_3*sin(tc),'--', ...
     'Color',[0.15 0.15 0.15],'LineWidth',1.2, ...
     'DisplayName','Obstacle-3 exact CBF envelope');
plot(G.x_obs3+G.r_outer_3*cos(tc),G.y_obs3+G.r_outer_3*sin(tc),':', ...
     'Color',[0.05 0.05 0.05],'LineWidth',1.8, ...
     'DisplayName','Obstacle-3 outer-MPC circle');

plot(G.gateRear1(1),G.gateRear1(2),'s','MarkerSize',9, ...
     'MarkerEdgeColor',[0.10 0.45 0.80], ...
     'MarkerFaceColor',[0.55 0.80 1.00],'LineWidth',1.2, ...
     'DisplayName','AVOID gates');
plot(G.gateRear2(1),G.gateRear2(2),'s','MarkerSize',9, ...
     'MarkerEdgeColor',[0.10 0.45 0.80], ...
     'MarkerFaceColor',[0.55 0.80 1.00],'LineWidth',1.2, ...
     'HandleVisibility','off');

plot(G.x_goal,G.y_goal,'kp','MarkerSize',13, ...
     'MarkerFaceColor',[0.95 0.90 0.10],'LineWidth',1.4, ...
     'DisplayName','Parking goal (44.5,21)');

plot(G.x_Sstart,G.yLane,'gs','MarkerSize',11, ...
     'MarkerFaceColor',[0.20 0.80 0.20], ...
     'DisplayName','Reverse-S start (60,16.36)');

if ~isempty(trajM)
    plot(trajM(:,2),trajM(:,3),'-','Color',refCol,'LineWidth',2.5, ...
         'DisplayName','Reference trajectory');
end

end


function [m,i] = minNonempty(v)
if isempty(v)
    m = NaN;
    i = 1;
else
    [m,i] = min(v);
end
end


function a = wrapToPiLocal(a)
a = mod(a+pi,2*pi) - pi;
end


function s = modeNameLocal(m)
switch m
    case 1, s = 'TRACK';
    case 2, s = 'AVOID';
    case 3, s = 'REJOIN';
    case 4, s = 'STOP@S';
    case 5, s = 'REV-S';
    case 6, s = 'PARKED';
    case 7, s = 'ESCAPE';
    case 8, s = 'SAFE-HOLD';
    otherwise, s = sprintf('mode %d',m);
end
end
