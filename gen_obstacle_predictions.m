clc;clear all;close all;
%% tuning para
xi_bar_x = 0.5;
xi_bar_y = 0.5;
M=100;
Delta_cw = 1.5;
start = 7;
show_k = 40;
%% load obs traj
load('log330.mat')
% load('obs_traj.mat')
X_obs = obs_pos(start:end,1)-obs_pos(start,1);
Y_obs = obs_pos(start:end,2);
simulength = length(X_obs);

%% Observer design
% para
lr = 1.45;
lfr = 2.8;
Ts = 0.1;
C = [1 0 0 0;
     0 1 0 0;
     0 0 0 1];
alpha = 0.01;
% vertex for LPV observer
delta_f(1) = pi/5;
delta_f(2) = -pi/5;
for i = 1:2
    beta(i) = atan((lr*tan(delta_f(i)))/(lfr));
end
V = [20 20];
psi = [-pi/3 pi/3];
% solve LMI 
P_sdp = sdpvar(4,4);
R_sdp = sdpvar(size(C,1),4);
gamma_sdp = sdpvar(1);
lambda_sdp = sdpvar(1);

F = [];
F = [F,P_sdp>=eye(4)];
F = [F,gamma_sdp>=0];
F = [F,lambda_sdp>=0];
F = [F,lambda_sdp<=0.9];
F = [F,[P_sdp R_sdp';
        R_sdp gamma_sdp*eye(size(C,1))]>=0];
for i  = 1:2
    for j = 1:2
        for k = 1:2
            A    = [0 0 cos(psi(k)+beta(j)) -V(i)*sin(psi(k)+beta(j));
                    0 0 sin(psi(k)+beta(j))  V(i)*cos(psi(k)+beta(j));
                    0 0 0 0;
                    0 0 cos(beta(j))*tan(delta_f(j))/lfr 0];
            F = [F,A'*P_sdp+P_sdp*A-C'*R_sdp-R_sdp'*C+2*alpha*P_sdp<=0];
            F = [F,[A'*P_sdp+P_sdp*A-C'*R_sdp-R_sdp'*C+eye(4) -R_sdp'*C;
                    -C'*R_sdp -lambda_sdp^2*eye(4)]<=0];
        end
    end
end

optimize(F,gamma_sdp);

P_fea = value(P_sdp);
R_fea = value(R_sdp);
gamma_fea = value(gamma_sdp);
lambda_fea = value(lambda_sdp);

L = inv(P_fea)*(R_fea');
P_11 = P_fea(1:2,1:2);
P_21 = P_fea(3:4,1:2);
P_22 = P_fea(3:4,3:4);
P_e = P_11-P_21'*inv(P_22)*P_21;

%% generate observed state
X_hat_obs = X_obs(1);
Y_hat_obs = Y_obs(1);
V_hat_obs = 25;
psi_hat_obs = 0;
x_hat_obs = [X_hat_obs;Y_hat_obs;V_hat_obs;psi_hat_obs];

X_hat_obs_list = [];
Y_hat_obs_list = [];
psi_hat_obs_list = [];
V_hat_obs_list = [];
vx_hat_obs_list = [];
vy_hat_obs_list = [];

for j = 1:simulength

fx_hat = [V_hat_obs*cos(psi_hat_obs);
          V_hat_obs*sin(psi_hat_obs);
          0;
          V_hat_obs/(lfr)*tan(0*pi/180)];

y = [X_obs(j);Y_obs(j);0];

x_hat_dot = fx_hat+L*(y-C*x_hat_obs);
x_hat_obs = x_hat_obs + Ts.*x_hat_dot;

X_hat_obs = x_hat_obs(1);
Y_hat_obs = x_hat_obs(2);
V_hat_obs = x_hat_obs(3);
psi_hat_obs = x_hat_obs(4);

X_hat_obs_list = [X_hat_obs_list X_hat_obs];
Y_hat_obs_list = [Y_hat_obs_list Y_hat_obs];
V_hat_obs_list = [V_hat_obs_list V_hat_obs];
psi_hat_obs_list = [psi_hat_obs_list psi_hat_obs];
vx_hat_obs_list = [vx_hat_obs_list x_hat_dot(1)];
vy_hat_obs_list = [vy_hat_obs_list x_hat_dot(2)];
end

%% plot 
co = get(gca,'colororder');close;
blue = co(1,:);
orange = co(2,:);
yellow = co(3,:);
purple = co(4,:);
green = co(5,:);
lightblue = co(6,:);
red = co(7,:);
% traj
figure 
plot(X_obs,Y_obs, '.','color',red,'LineWidth',4);
hold on;
plot(X_hat_obs_list,Y_hat_obs_list, '.','color',blue,'LineWidth',4);
hold on;
% axis equal
grid on
% velocity
plot_t = [1:simulength];
figure 
plot(plot_t,V_hat_obs_list, '-','color',red,'LineWidth',4);
hold on;
plot(plot_t,vx_hat_obs_list, '-','color',blue,'LineWidth',4);
hold on;
plot(plot_t,vy_hat_obs_list, '-','color',green,'LineWidth',4);
hold on;
grid on

%% reachable set: local
A = [1 0; 0 1];
B = [Ts 0; 0 Ts];
A_xi = [1 0;-1 0;0 1;0 -1]; 
b_xi = [xi_bar_x;xi_bar_x;xi_bar_y;xi_bar_y];

P_xi = Polyhedron(A_xi,b_xi);
Px = Polyhedron(A_xi,Ts*b_xi);
position = poly2pos(Px);
pos_pred(1,:) = [0 0 0 0];
pos_pred(2,:) = position;
for j = 3:M

         Px = A*Px+B*P_xi;
         position = poly2pos(Px);
         pos_pred(j,:) = position;

end

figure
for i = 1:M
         rectangle('Position',pos_pred(i,:));
         hold on;
end
axis equal

%% reachable set: with velocity
pos_term = pos_pred;
x_obs = [X_hat_obs_list(1);Y_hat_obs_list(1)];

for i = 1:simulength

    x_obs = [X_hat_obs_list(i);Y_hat_obs_list(i)];
    u_current = [vx_hat_obs_list(i);vy_hat_obs_list(i)];

    l = 1; 
    x_obs_pred = x_obs;
    active_index = [];


    for j = 1:M

         pos_term(j,1) = pos_pred(j,1)+x_obs_pred(1);
         pos_term(j,2) = pos_pred(j,2)+x_obs_pred(2);

         % active index
         y_pred_low = pos_term(j,2);
         y_pred_high = pos_term(j,2)+pos_term(j,4);
         if (y_pred_high >= -Delta_cw) && (y_pred_low <= Delta_cw)  
             active_index(l) = j;
             l = l + 1;
         end

         % update 
         x_obs_pred = A*x_obs_pred+B*u_current;

    end

    % log
    w_obs_list(:,i) = x_obs;
    position_list{i} = pos_term;
    Index{i} = active_index;


end

%% save 
for i = [1:40 100]
    Index{1, i}  = [];
end
for i = [41:81]
    if isempty(Index{1, i}) == 1
        Index{1, i} = Index{1, i-1};
    else
    end
end
filename = "obs_traj_pred";
save(filename,"w_obs_list","position_list","Index");

%% plot 
co = get(gca,'colororder');close;
blue = co(1,:);
red = co(7,:);
% traj
position_plot = position_list{show_k};
figure 
plot(X_obs,Y_obs, '.','color',red,'LineWidth',4);
hold on;
plot(w_obs_list(1,:),w_obs_list(2,:), '.','color',blue,'LineWidth',4);
hold on;
for i = 1:M
    rectangle('Position',position_plot(i,:));
    hold on;
end
axis equal
grid on

% functions
function position = poly2pos(Px)
         Px_para = Px.b;
         Px_left = [-Px_para(1) -Px_para(3)];
         Px_width = 2*Px_para(1);
         Px_height = 2*Px_para(3);
         position = [Px_left Px_width Px_height];
end