 function vehicle_safe_mpftc(xInit, save_file, sim_time,w_obs_list, position_list, Index)

  import casadi.*
  
  % obs
  distance = 1000;

  % State reference
  v_ref = 20;
  x_ref = [0;0;0;0;0;v_ref;0];
 
  % Vehicle model parameters
  w_0 = 20;
  w_1 = 0.9;
  t_acc = 1.8;
  l = 3;
  d0 = 15;
  
  % Vehicle constraints
  ey_bar = 0.4;
  e_psi_bar = 0.61;
  delta_bar = 0.53;
  v_bar = 30;
  alpha_bar = 0.35;
  a_bar_low_1 = -3;
  a_bar_high_1 = 3;
  a_bar_low_2 = -4;
  a_bar_high_2 = 4;
  a_bar_low_3 = -5;
  a_bar_high_3 = 5;
  a_bar_low = a_bar_low_1;
  a_bar_high = a_bar_high_1;
  
  % Horizon and shooting intervals
  M = 100;
  N = 10;
  ts = 0.1;

  % Closed loop iterations
  kIter = sim_time/ts;

  % Cost tuning
  Q = diag([0 1 1 10 1 1 1]);
  R = diag([4 10]);
  
  % Compute terminal cost
  P_lat = [325.51 593.13 97.32 1.46;
           593.13 6091.11 1979.43 29.75;
           97.32 1979.43 1159.47 17.15;
           1.46 29.75 17.15 1.28];
  P_lon = [210.78 80.19;
           80.19 38.29];
  P = blkdiag(0,P_lat,P_lon);

  % Define states and controls
  xLabels = {'s','e_y','e_psi','delta','alpha','v','a'};
  uLabels = {'a_req','delta_sp'};

  % Indexing functions for states
  nx = numel(xLabels);
  nu = numel(uLabels);
  for k = 1:nx
      xI.(char(xLabels{k})) = k;
  end
  for k = 1:nu
      uI.(char(uLabels{k})) = k;
  end
  xuI.x = xI;
  xuI.u = uI;

  % States and controls
  x = SX.sym('x',[numel(xLabels),1]);
  u = SX.sym('u',[numel(uLabels),1]);
                         
  % Define the right hand side
  rhs = [ x(xI.v)*cos(x(xI.e_psi));
          x(xI.v)*sin(x(xI.e_psi));
          x(xI.v)*(1/l)*tan(x(xI.delta));
          x(xI.alpha);
          w_0^2*(u(uI.delta_sp)-x(xI.delta))-2*w_0*w_1*x(xI.alpha);
          x(xI.a);
          t_acc*(u(uI.a_req)-x(xI.a));
          ];

  % Define integrator
  n_int_steps = 10;
  rk4  = casadi_rk4(x,u,rhs,ts,n_int_steps,'rk4');

  % Create OCP variables
  V  = MX.sym('V',  M*nu + (M+1)*nx );
  Vr = MX.sym('Vr', M*nu + (M+1)*nx );

  % Indexing of variables
  [iV, iif] = state_indexing(xLabels,uLabels,M,xuI);

  % Define cost and constraints
  cost = 0;
  g = [];
  iG_.dyn = {}; iG_.xf = {}; iG_.stop = {};
  offset = 0;

  % Loop over cost for 1 : N
  for k = 1:M
    
    % Integrator constraint
    [X_,] = rk4(V(iV('x',k)),V(iV('u',k)));
    g = [g; V(iV('x',k+1)) - X_];
    iG_.dyn{k} = (1:nx).' + offset;
    offset = offset + nx;
    
    % Obstacle constraint
    g = [g; V(iV('x',k,'s'))-Vr(iV('x',k,'s'))];
    iG_.obs{k} = 1 + offset;
    offset = offset + 1;
     
    % States at time k
    xk = V(iV('x',k));
    
    % Control inputs at time k
    uk = V(iV('u',k));
    
    % Reference at time k
    xkr = x_ref;
    
    % Cost for times [0, N]
    if k < N +1
      dx = xk - xkr;
      du = uk;
      cost = cost + dx'*Q*dx + du'*R*du;
    end
      
  end

  % Terminal states
  xN = V(iV('x',N+1));

  % Terminal reference
  xrN = xkr;

  % Terminal cost
  cost = cost + (xN-xrN)'*P*(xN-xrN);
  
  % Terminal set constraint 
  K_lon = [0.0693 0.4151];
  H_lon = [1 0;0 1;0 -1;1 1;-2 -1];
  b_lon = [1.3889;1;4;1.4;32];
  for k = N + 1 : M + 1
    xM = V(iV('x',k));
    rM = x_ref(6:7);
    g = [g; K_lon*(xM(6:7)-rM)];
    iG_.xf{k-N} = 1 + offset;
    offset = 1 + offset;
  end
  
  % Terminal safe set constraint
  g = [g; V(iV('x',M+1,'v'))];
  iG_.stop{1} = 1 + offset;
  offset = 1 + offset;

  % Set up nlp
  nlp = struct( 'x', V, 'p', Vr, 'f', cost, 'g', g);
  opts = struct('ipopt',struct());
  opts.ipopt.print_level = 0;
  opts.ipopt.suppress_all_output = 'yes';
  opts.print_time = 0;
  solver = nlpsol( 'solver', 'ipopt', nlp, opts );

  % Set up constraints indexing
  iG =  constraint_indexing(iG_,iif);

  % Set  up constraints
  ng = size(g);
  lbv = -inf*ones(size(V));
  ubv =  inf*ones(size(V));
  lbg = -inf*ones(ng);
  ubg =  inf*ones(ng);

  % Fix dynamics
  lbg(iG('dyn')) = zeros( size(iG('dyn')) );
  ubg(iG('dyn')) = zeros( size(iG('dyn')) );

  % Terminal set
  lbg(iG('xf')) = a_bar_low_1*ones(size(iG('xf')) );
  ubg(iG('xf')) =  a_bar_high_1*ones(size(iG('xf')) );
  
  lbg(iG('stop')) = 0*ones( size(iG('stop')) );
  ubg(iG('stop')) = 0*ones( size(iG('stop')) );

  % Obstacle
  ubg(iG('obs')) = zeros( size(iG('obs')) );

  % State and control constraints
  lbv(iV('x',1:M,'e_y')) = -ey_bar*ones(size(iV('x',1:M,'e_y')));
  ubv(iV('x',1:M,'e_y')) = ey_bar*ones(size(iV('x',1:M,'e_y')));
  lbv(iV('x',1:M,'e_psi')) = -e_psi_bar*ones(size(iV('x',1:M,'e_psi')));
  ubv(iV('x',1:M,'e_psi')) = e_psi_bar*ones(size(iV('x',1:M,'e_psi')));
  lbv(iV('x',1:M,'delta')) = -delta_bar*ones(size(iV('x',1:M,'delta')));
  ubv(iV('x',1:M,'delta')) = delta_bar*ones(size(iV('x',1:M,'delta')));
  lbv(iV('x',1:M,'alpha')) = -alpha_bar*ones(size(iV('x',1:M,'alpha')));
  ubv(iV('x',1:M,'alpha')) = alpha_bar*ones(size(iV('x',1:M,'alpha')));
  lbv(iV('x',1:M,'v')) = zeros(size(iV('x',1:M,'v')));
  ubv(iV('x',1:M,'v')) = v_bar*ones(size(iV('x',1:M,'v')));
  lbv(iV('u',1:M,'delta_sp')) = -delta_bar*ones(size(iV('u',1:M,'delta_sp')));
  ubv(iV('u',1:M,'delta_sp')) = delta_bar*ones(size(iV('u',1:M,'delta_sp')));

  % Initialization for solver
  Vref = zeros(size(V)); 
  x0 = zeros(size(V));

  % Apply initial state
  x0(iV('x')) = repmat(xInit,M+1,1);
  x_0 = xInit;

  % Logging variables
  X = x_0; 
  U = []; 
  P_obs = []; Px_pred_list = []; 
  XS_open = []; XV_open = []; XA_open = []; 
  XEy_open = []; XEpsi_open = []; XDelta_open = []; 
  UDelta_open = []; UA_open = [];
  S_1 = [];S_2 = [];S_3 = [];
  A = []; t_QP = [];
  
  % Closed loop simulation
  fprintf('Processing:\n'); percent = [];
  t_elapsed = 0;
  time_array = zeros(1,kIter);
  for k = 1 : kIter
    tic;
    
    active_index = Index{k};
    xt = [x_0(1);x_0(6);x_0(7)];

    if isempty(active_index) == 1

        Vref(iV('x',1:M,'s')) = distance;
        
    else

        Vref(iV('x',1:M,'s')) = distance;
        obs_pos = position_list{1, k}(active_index(1):active_index(end),1)-d0;
        Vref(iV('x',active_index(1):active_index(end),'s')) = obs_pos;

        [Sval_min] = gen_Sval(t_acc,ts,v_bar,a_bar_low,a_bar_high,...
                                    active_index(1),xt);
        if Sval_min >= obs_pos(1)
            [Sval_min_2] = gen_Sval(t_acc,ts,v_bar,a_bar_low_2,a_bar_high_2,...
                                    active_index(1),xt);
            if Sval_min_2 >= obs_pos(1)
                a_bar_high = a_bar_high_2;
                a_bar_low = a_bar_low_2;
            else
                a_bar_high = a_bar_high_3;
                a_bar_low = a_bar_low_3;
            end
        else
        end

    end

    % obstain delta for each driving mode
    [Sval_list_1] = gen_Delta(t_acc,ts,v_bar,a_bar_low_1,a_bar_high_1,xt,M);
    [Sval_list_2] = gen_Delta(t_acc,ts,v_bar,a_bar_low_2,a_bar_high_2,xt,M);
    [Sval_list_3] = gen_Delta(t_acc,ts,v_bar,a_bar_low_3,a_bar_high_3,xt,M);

    % adaptive a priori known constraint for acceleration
    lbv(iV('x',1:M,'a')) = a_bar_low*ones(size(iV('x',1:M,'a')));
    ubv(iV('x',1:M,'a')) = a_bar_high*ones(size(iV('x',1:M,'a')));
    lbv(iV('u',1:M,'a_req')) = a_bar_low*ones(size(iV('u',1:M,'a_req')));
    ubv(iV('u',1:M,'a_req')) = a_bar_high*ones(size(iV('u',1:M,'a_req')));
    
    % Update initial state
    lbv(iV('x',1)) = x_0;
    ubv(iV('x',1)) = x_0;
  
    % Solve problem
    sol = solver('x0',x0,'lbx',lbv,'ubx',ubv,'lbg',lbg,'ubg',ubg,'p',Vref);
    if isequal(solver.stats.return_status,'Infeasible_Problem_Detected')
      disp('Infeasible');
      break;
    end
    v_opt = full(sol.x); 
    x0 = v_opt;

    % Get control input and integrate the closed loop system
    u_0 = full(v_opt(iV('u',1)));
    x_0 = full(rk4(x_0,u_0));
  
    % Fill logging vector
    X = [X x_0];
    U = [U u_0];
    P_obs = [P_obs;
             w_obs_list(1,k) w_obs_list(2,k)];
    XEy_open = [XEy_open; v_opt(iV('x',1:M+1,'e_y'))'];
    XEpsi_open = [XEpsi_open; v_opt(iV('x',1:M+1,'e_psi'))'];
    XS_open = [XS_open; v_opt(iV('x',1:M+1,'s'))'];
    XV_open = [XV_open; v_opt(iV('x',1:M+1,'v'))'];
    XA_open = [XA_open; v_opt(iV('x',1:M+1,'a'))'];
    XDelta_open  = [XDelta_open; v_opt(iV('x',1:M+1,'delta'))'];
    UA_open = [UA_open; v_opt(iV('u',1:M,'a_req'))'];
    UDelta_open = [UDelta_open; v_opt(iV('u',1:M,'delta_sp'))'];
    A = [A a_bar_low];
    S_1{k} = Sval_list_1;
    S_2{k} = Sval_list_2;
    S_3{k} = Sval_list_3;

    % Timing and printing
    t1 = toc;
    time_array(k) = t1;
    t_elapsed = t_elapsed + t1;
    eta = (kIter-k) * t1;
    fprintf(repmat('\b',1,length(percent)-1));
    percent = ...
      sprintf('%.2f%%%% - Est. time left: %.2fs - Time elapsed: %.2fs',...
      (100*k/kIter),eta,t_elapsed);
    fprintf(percent); drawnow;

    t_QP = [t_QP t1];
end
fprintf('\n');


tx = 0:ts:(k)*ts;

save(save_file,'X','U','ts','tx','sim_time','P_obs','Px_pred_list',...
               'XS_open','XV_open','XEy_open','XA_open', 'XEpsi_open', 'XDelta_open',...
               'UA_open','UDelta_open','M','S_1','S_2','S_3','A','t_QP')

