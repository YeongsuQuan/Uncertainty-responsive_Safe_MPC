function [Px,Py,boundary_x,boundary_y,plot_body,plot_direction,plot_tire]=draw_veh_cg_steer(xi,yi,yaw,width,length,color,line_w,steer_angle)
% [Px,Py,boundary_x,boundary_y,plot_body,plot_direction,plot_tire] = draw_veh_rear_steer(xi,yi,yaw,width,length,color,line_w,steer_angle)
%  x : [m]
%  y : [m]
%  yaw : [rad]
%  width : [m]
%  length : [m]
%  color : rgb
%  linewidth
%  steering angle : [rad]

yaw = yaw-pi/2;

if width>1
    w=width;
    l=length;
end
x=0;
y=0;


% center of gravity
CG=[x;y];

% vehicle body
P1=[x+w/2; y-l/2];
P2=[x+w/2; y+l/2];
P3=[x-w/2; y+l/2];
P4=[x-w/2; y-l/2];

% tire
T1_P1=[P1(1,1)+w/15; P1(2,1)-l/10+l/5];
T1_P2=[P1(1,1)+w/15; P1(2,1)+l/10+l/5];
T1_P3=[P1(1,1)-w/15; P1(2,1)+l/10+l/5];
T1_P4=[P1(1,1)-w/15; P1(2,1)-l/10+l/5];
T1_xy=[P1(1,1); P1(2,1)-l/5];

T2_P1=[P2(1,1)+w/15; P2(2,1)-l/10-l/5];
T2_P2=[P2(1,1)+w/15; P2(2,1)+l/10-l/5];
T2_P3=[P2(1,1)-w/15; P2(2,1)+l/10-l/5];
T2_P4=[P2(1,1)-w/15; P2(2,1)-l/10-l/5];
T2_xy=[P2(1,1); P2(2,1)-l/5];

T3_P1=[P3(1,1)+w/15; P3(2,1)-l/10-l/5];
T3_P2=[P3(1,1)+w/15; P3(2,1)+l/10-l/5];
T3_P3=[P3(1,1)-w/15; P3(2,1)+l/10-l/5];
T3_P4=[P3(1,1)-w/15; P3(2,1)-l/10-l/5];
T3_xy=[P3(1,1); P3(2,1)-l/5];

T4_P1=[P4(1,1)+w/15; P4(2,1)-l/10+l/5];
T4_P2=[P4(1,1)+w/15; P4(2,1)+l/10+l/5];
T4_P3=[P4(1,1)-w/15; P4(2,1)+l/10+l/5];
T4_P4=[P4(1,1)-w/15; P4(2,1)-l/10+l/5];
T4_xy=[P1(1,1); P1(2,1)-l/5];

% vehicle direction
D_P1=[x+w/2; y+l/4];
D_P2=[x; y+l/2];
D_P3=[x-w/2; y+l/4];


P_set=[CG P1 P2 P3 P4 T1_P1 T1_P2 T1_P3 T1_P4 T2_P1 T2_P2 T2_P3 T2_P4 T3_P1 T3_P2 T3_P3 T3_P4 T4_P1 T4_P2 T4_P3 T4_P4 D_P1 D_P2 D_P3];
T_set=[T1_xy T2_xy T3_xy T4_xy];
T=[cos(-yaw) sin(-yaw); -sin(-yaw) cos(-yaw)];
T_tire=[cos(-steer_angle) sin(-steer_angle); -sin(-steer_angle) cos(-steer_angle)];
P_set_n=T*P_set;
T_set_n=T*T_set;

P_set_n(1,:)=P_set_n(1,:)+xi;
P_set_n(2,:)=P_set_n(2,:)+yi;
T_set_n(1,:)=T_set_n(1,:)+xi;
T_set_n(2,:)=T_set_n(2,:)+yi;

xx=[P_set_n(1,2:5) P_set_n(1,2)];
yy=[P_set_n(2,2:5) P_set_n(2,2)]; 


CGx=[P_set_n(1,1)];
CGy=[P_set_n(2,1)];

T1xx=P_set_n(1,6:9);
T1yy=P_set_n(2,6:9); 

P_set_n(:,10:13)=T_tire*(P_set_n(:,10:13)-T_set_n(:,2))+T_set_n(:,2); 
T2xx=P_set_n(1,10:13); 
T2yy=P_set_n(2,10:13);

P_set_n(:,14:17)=T_tire*(P_set_n(:,14:17)-T_set_n(:,3))+T_set_n(:,3); 
T3xx=P_set_n(1,14:17);
T3yy=P_set_n(2,14:17); 

T4xx=P_set_n(1,18:21);
T4yy=P_set_n(2,18:21); 


Dxx=[P_set_n(1,22:24) P_set_n(1,22)];
Dyy=[P_set_n(2,22:24) P_set_n(2,22)];

plot_body=plot(xx,yy,'linewidth',line_w);
plot_body.Color = color;
hold on
plot_cg=plot(CGx,CGy,'o','linewidth',5);
plot_cg.Color = color;
hold on
plot_direction=plot(Dxx,Dyy,'linewidth',2);
plot_direction.Color = color;
hold on
plot_tire=fill(T1xx,T1yy,color);
hold on
plot_tire=fill(T2xx,T2yy,color);
hold on
plot_tire=fill(T3xx,T3yy,color);
hold on
plot_tire=fill(T4xx,T4yy,color);
hold on

Px=CGx;
Py=CGy;
boundary_x=xx;
boundary_y=yy;
end

