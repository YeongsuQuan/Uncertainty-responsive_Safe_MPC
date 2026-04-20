function [Sval] = gen_sval(t_acc,ts,v_bar,a_bar_low,a_bar_high,N,xt)


% System model - continuous time
Act     = [0 1 0;
           0 0 1;
           0 0 -t_acc];
Bct     = [0;0;t_acc];

Cct     = [1 0 0];

Dct     = zeros(1,1);

% Discrete-time model with zoh
Model           =   c2d(ss(Act,Bct,Cct,Dct),ts);  

% System model matrices (discrete time):
[A,B,C,D]       =   ssdata(Model);                  % Model matrices 

% Signal dimensions

nx      =   size(A,1);      % number of states
nu      =   size(B,2);     % number of inputs

% Input inequality constraints 

Cu      =   [1;-1];
du      =   [a_bar_high;-a_bar_low];
nqu     =   size(Cu,1);                 % Number of input inequality constraints per stage

% State inequalities 
Cx     =   [0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
dx     =   [v_bar;0;a_bar_high;-a_bar_low];
nqx    =   size(Cx,1);

% Build overall matrices and vectors for LP 

[~,~,Lambda_x,Gamma_x]     =   traj_matrices(N,A,B,C,D);
Cubar                                   =   zeros(N*nqu,N*nu);
dubar                                   =   zeros(N*nqu,1);
Cxbar                                   =   zeros((N+1)*nqx,(N+1)*nx);
dxbar                                   =   zeros((N+1)*nqx,1);
C_bar                                   =   zeros(nu,(N+1)*nx);

for ind = 1:N+1
    Cxbar((ind-1)*nqx+1:ind*nqx,(ind-1)*nx+1:ind*nx)        =   Cx;
    dxbar((ind-1)*nqx+1:ind*nqx,1)                          =   dx;
end

for ind = 1:N
    Cubar((ind-1)*nqu+1:ind*nqu,(ind-1)*nu+1:ind*nu)        =   Cu;
    dubar((ind-1)*nqu+1:ind*nqu,1)                          =   du;
end


% Constraints
Aineq   =   [Cubar;Cxbar*Gamma_x];
bineq   =   [dubar;-Cxbar*Lambda_x*xt+dxbar];

% Cost
C_bar(nu,N*nx+1) = 1;
f_bar = C_bar*Gamma_x;

% LP
options = optimoptions('linprog','Display','none');
U = linprog(f_bar',Aineq,bineq,[],[],[],[],options);

% Opt val
X = Lambda_x*xt+Gamma_x*U;
Sval = C_bar*X;

end

