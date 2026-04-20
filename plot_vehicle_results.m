clc;clear;close all;

fprintf('Generating plots for vehicle example:\n');

third_party_plot = false;
if exist('tightfig.m','file') && exist('tight_subplot.m','file')
  third_party_plot = true;
end

show_list = [57 70 90];
filename_traj = 'traj_ZOD_';
filename_a = 'a_ZOD.eps';
filename_a_req = 'a_req_ZOD.eps';
filename_v = 'v_ZOD.eps';
filename_s1 = 'x1_ZOD.eps';
filename_s2 = 'x2_mode_ZOD.eps';
filename_a_areq = 'a_a_req_ZOD.eps';
path_fig = 'figures/';  % update this path to save figures elsewhere

ey_bar = 0.4;
e_psi_bar = 0.61;
delta_bar = 0.53;
v_bar = 30;
alpha_bar = 0.35;
rw = 3.5;
Delta_cw = 1.5;
edge = 500;
start_point = 100;
end_point = 200;

co = get(gca,'colororder');close;
blue = co(1,:);
orange = co(2,:);
yellow = co(3,:);
purple = co(4,:);
green = co(5,:);
lightblue = co(6,:);
red = co(7,:);
black = [0 0 0];
predlw = 6;
trajlw = 3;
roadlw = 3;
plotlw = 4;
conslw = 6;
framelw = 6;

% Load data
load('obs_traj_pred.mat');
addpath(genpath('.'));
safe_mpftc = load(gen_path(...
         {'data','vehicle','vehicle_safe_mpftc.mat'})); 
ts = safe_mpftc.ts;
t_QP = safe_mpftc.t_QP;
M = safe_mpftc.M;
XA_open = safe_mpftc.XA_open;
XS_open = safe_mpftc.XS_open;
XV_open = safe_mpftc.XV_open;
XEy_open = safe_mpftc.XEy_open; 
sim_time = safe_mpftc.sim_time;
log_leng = length(safe_mpftc.P_obs(:,1));
tx = safe_mpftc.tx(1:log_leng);
X = safe_mpftc.X(:,1:log_leng);
U = safe_mpftc.U(:,1:log_leng);
P_obs = w_obs_list';
a_bar_low = safe_mpftc.A(:,1:log_leng);
a_bar_high = -a_bar_low;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% traj
% % road
% k = 1;
% nPlots = 7;
% data{k}.nPlots = nPlots;
% data{k}.x = {[0 edge];[0 edge];[0 edge];...
%              [0 edge];[0 edge];[0 edge];[0 edge]};
% data{k}.y = {[rw/2 rw/2];[-rw/2 -rw/2];[0 0];...
%              [3*rw/2 3*rw/2];[-3*rw/2 -3*rw/2];...
%              [rw rw];[-rw -rw]};
% data{k}.color = {purple;purple;purple;...
%                  purple;purple;purple;purple};
% data{k}.linestyle = {'-';'-';'--';'-';'-';'--';'--'};
% data{k}.linewidth = {roadlw;roadlw;roadlw;roadlw;roadlw;roadlw;roadlw};
% data{k}.title = '';
% data{k}.xlabel = '$X$ [m]'; data{k}.ylabel = '$Y$ [m]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'','','','','','','','','','',''};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% 
% 
% for k = 1:length(show_list)
% gh=figure; 
% set(gh,'position',[px(1) px(2) px(3)*1.2 px(4)/3]);
% axe = plot_trajectories(data,opts);
% set(gcf,'currentaxes',axe(1))
% show_k = show_list(k);
% % road
% rectangle('Position',...
%     [-1,-Delta_cw,edge,2*Delta_cw],...
%     'FaceColor',lightblue,'EdgeColor',lightblue,'FaceAlpha',.3,'LineWidth',roadlw)
% % ego vehicle
% plot(X(1,1:show_k), X(2,1:show_k), '.-','color',blue,'LineWidth',trajlw);
% hold on;
% draw_veh_cg_steer(X(1,show_k),X(2,show_k),0,2,4.5,red,trajlw,0);
% hold on;
% % ego vehicle open-loop traj
% patch(XS_open(show_k,:),XEy_open(show_k,:),XV_open(show_k,:),...
%     'EdgeColor','interp','Marker','o','MarkerSize',predlw,'MarkerFaceColor','flat');
% colorbar;
% hold on;
% % obs open-loop traj
% for i = 1:M
%     rectangle('Position',...
%     position_list{1, show_k-1}(i,:),...
%     'FaceColor',orange,'EdgeColor','none','FaceAlpha',.3,'LineWidth',roadlw)
% hold on;
% end
% for i = 1:M
%     rectangle('Position',...
%     position_list{1, show_k}(i,:),...
%     'FaceColor',green,'EdgeColor','none','FaceAlpha',.3,'LineWidth',roadlw)
% hold on;
% end
% % obs traj
% plot(P_obs(1:show_k,1), P_obs(1:show_k,2), '.-','color',orange,'LineWidth',trajlw);
% hold on;
% % obs vehicle
% draw_veh_cg_steer(P_obs(show_k,1),P_obs(show_k,2),0,2,4.5,black,trajlw,0);
% hold on;
% % time stamp
% txt = ['t = ',num2str((show_k-1)*ts),'s'];
% text(start_point+5,7,txt,'FontSize',35);
% % set axis and text
% axis equal
% xlim([start_point end_point]);
% ylim([-10 10]);
% set(axe(1).Legend,'location','northeast');
% box on
% ax = gca;
% ax.LineWidth = framelw;
% filename = [filename_traj,num2str(k)];
% saveas(gcf,[path_fig filename],'epsc');
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% a
% k = 1;
% nPlots = 3;
% data{k}.nPlots = nPlots;
% data{k}.x = {tx;tx;tx}; 
% data{k}.y = {X(7,:);a_bar_high;a_bar_low};
% data{k}.color = {black;red;red};
% data{k}.linestyle = {'-';'-';'-'};
% data{k}.linewidth = {plotlw;conslw;conslw;};
% data{k}.title = '';
% data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$a$ [m/$s^2$]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'$a$','',''};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% gh=figure; 
% set(gh,'position',[px(1) px(2) px(3) px(4)/3]);
% axe = plot_trajectories(data,opts);
% 
% axes(axe(1)); 
% xlim([0 tx(end)]);
% ylim([-5 5]);
% set(axe(1).Legend,'location','northeast')
% box on
% ax = gca;
% ax.LineWidth = framelw;
% saveas(gcf,[path_fig filename_a],'epsc');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% a req
% k = 1;
% nPlots = 3;
% data{k}.nPlots = nPlots;
% data{k}.x = {tx;tx;tx}; 
% data{k}.y = {U(1,:);a_bar_high;a_bar_low};
% data{k}.color = {black;red;red};
% data{k}.linestyle = {'-';'-';'-'};
% data{k}.linewidth = {plotlw;conslw;conslw;};
% data{k}.title = '';
% data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$a_{req}$ [m/$s^2$]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'$a_{req}$','',''};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% gh=figure; set(gh,'position',[px(1) px(2) px(3) px(4)/3]);
% axe = plot_trajectories(data,opts);
% 
% axes(axe(1)); 
% xlim([0 tx(end)]);
% ylim([-5 5]);
% set(axe(1).Legend,'location','northeast')
% box on
% ax = gca;
% ax.LineWidth = framelw;
% saveas(gcf,[path_fig filename_a_req],'epsc');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% v
% k = 1;
% nPlots = 3;
% data{k}.nPlots = nPlots;
% data{k}.x = {[0 tx(end)];[0 tx(end)];tx;}; 
% data{k}.y = {[v_bar v_bar];[0 0];X(6,:);};
% data{k}.color = {red;red;black};
% data{k}.linestyle = {'-';'-';'-'};
% data{k}.linewidth = {conslw;conslw;plotlw;};
% data{k}.title = '';
% data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$v$ [m/s]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'','','$v$'};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% gh=figure; set(gh,'position',[px(1) px(2) px(3) px(4)/3]);
% axe = plot_trajectories(data,opts);
% 
% axes(axe(1)); 
% xlim([0 tx(end)]);
% ylim([0-1 v_bar+1]);
% set(axe(1).Legend,'location','northeast')
% box on
% ax = gca;
% ax.LineWidth = framelw;
% saveas(gcf,[path_fig filename_v],'epsc');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% a
% k = 1;
% nPlots = 4;
% data{k}.nPlots = nPlots;
% data{k}.x = {tx;tx;tx;tx}; 
% data{k}.y = {U(1,:);X(7,:);a_bar_high;a_bar_low};
% data{k}.color = {blue;black;red;red};
% data{k}.linestyle = {'-';'-';'-';'-'};
% data{k}.linewidth = {plotlw;plotlw;conslw;conslw;};
% data{k}.title = '';
% data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = '$a$ [m/$s^2$]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'$a_{req}$','$a$','',''};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% gh=figure; 
% set(gh,'position',[px(1) px(2) px(3) px(4)/3]);
% axe = plot_trajectories(data,opts);
% 
% axes(axe(1)); 
% xlim([0 tx(end)]);
% ylim([-5 5]);
% set(axe(1).Legend,'location','northeast')
% box on
% ax = gca;
% ax.LineWidth = framelw;
% saveas(gcf,[path_fig filename_a_areq],'epsc');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% computation time
% k = 1;
% nPlots = 1;
% data{k}.nPlots = nPlots;
% data{k}.x = {[1:1:length(t_QP)]}; 
% data{k}.y = {t_QP};
% data{k}.color = {blue;black;red;red};
% data{k}.linestyle = {'o-';'-';'-';'-'};
% data{k}.linewidth = {plotlw;plotlw;conslw;conslw;};
% data{k}.title = '';
% data{k}.xlabel = 'Time insrance'; data{k}.ylabel = 'Run time [s]';
% data{k}.fontsize = 30;
% data{k}.gca_fontsize = 30;
% data{k}.grid ='on';
% data{k}.legend = {'','','',''};
% 
% opts.third_party = third_party_plot;
% opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
% opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
% px = get(0,'screensize');
% gh=figure; 
% set(gh,'position',[px(1) px(2) px(3) px(4)/3]);
% axe = plot_trajectories(data,opts);
% 
% axes(axe(1)); 
% xlim([0 length(t_QP)]);
% ylim([-0.1 5]);
% set(axe(1).Legend,'location','northeast')
% box on
% ax = gca;
% ax.LineWidth = framelw;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% g g_min
index_start = 42;
index_end = 81;
tx_active = tx(index_start:index_end-1);

for i = index_start:index_end

    s_m_min_1(i) =  safe_mpftc.S_1{1, i}(Index{1, i}(1));
    s_m_min_2(i) =  safe_mpftc.S_2{1, i}(Index{1, i}(1));
    s_m_min_3(i) =  safe_mpftc.S_3{1, i}(Index{1, i}(1));
    s_obs_m(i) =  position_list{i}(Index{1, i}(1))-14.8;
    s_m(i) =  XS_open(i,Index{1, i}(1));

end

g_k_m = s_m(1:end-1)-s_obs_m(1:end-1);
g_k_m_min_1 = s_m_min_1(1:end-1)-s_obs_m(1:end-1);
g_k_m_min_2 = s_m_min_2(1:end-1)-s_obs_m(1:end-1);
g_k_m_min_3 = s_m_min_3(1:end-1)-s_obs_m(1:end-1);
g_k_m_bar_1 = g_k_m-g_k_m_min_1;
g_k_m_bar_2 = g_k_m-g_k_m_min_2;
g_k_m_bar_3 = g_k_m-g_k_m_min_3;

g_k1_m = s_m(2:end)-s_obs_m(2:end);

k = 1;
nPlots = 4;
data{k}.nPlots = nPlots;
data{k}.x = {tx_active;tx_active;tx_active;tx_active}; 
data{k}.y = {g_k_m_bar_1(index_start:index_end-1);...
             g_k_m_bar_2(index_start:index_end-1);...
             g_k_m_bar_3(index_start:index_end-1);...
             g_k1_m(index_start:index_end-1)};
data{k}.color = {green;lightblue;blue;red};
data{k}.linestyle = {'-';'-';'-';'-';'-';'-';'-';'-'};
data{k}.linewidth = {plotlw;plotlw;plotlw;plotlw;plotlw;plotlw;plotlw;plotlw};
data{k}.title = '';
data{k}.xlabel = '$t$ [s]'; data{k}.ylabel = 'Traveled distance [m]';
data{k}.fontsize = 30;
data{k}.gca_fontsize = 30;
data{k}.grid ='on';
data{k}.legend = {'$\bar g^{1}_{k+m|k}$',...
                  '$\bar g^{2}_{k+m|k}$',...
                  '$\bar g^{3}_{k+m|k}$',...
                  '$g_{k+m|k+1}$'};

opts.third_party = third_party_plot;
opts.dim = [1 1]; opts.gap = [0.15 0.045]; 
opts.margh = [0.1 0.1]; opts.margw = [0.1 0.1];
px = get(0,'screensize');
gh=figure; set(gh,'position',[px(1) px(2) px(3) px(4)/2.5]);
axe = plot_trajectories(data,opts);
xlim([tx_active(1) tx_active(end)]);
set(axe(1).Legend,'location','southeast')
ax = gca;
ax.LineWidth = framelw;
