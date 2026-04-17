function [x_obs_next,y_obs_next,x_pred,active_index] = obs_state(M,ts,...
                                x_obs,y_obs,vx_obs,vy_obs,Delta_cw)
x = [x_obs,y_obs]';
u = [vx_obs,vy_obs]';

A = [1 0;
     0 1];
B = [ts 0;
     0 ts];

Px  = x;
j = 1;
active_index = [];

for i = 1:M+1
    x_pred(:,i) = Px;

    if (Px(2) >= -Delta_cw) & (Px(2) <= Delta_cw)  
        active_index(j) = i;
        j = j + 1;
    else
        j = j;
    end

    Px  = A*Px+B*u;
    
end

x_state_next = A*x+B*u;
x_obs_next = x_state_next(1);
y_obs_next = x_state_next(2);


end

