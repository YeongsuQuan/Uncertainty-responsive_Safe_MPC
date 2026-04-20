function [Sval_list] = gen_sval_list(t_acc,ts,v_bar,a_bar_low,a_bar_high,xt,M)

for N = 1:M
    [Sval] = gen_sval(t_acc,ts,v_bar,a_bar_low,a_bar_high,N,xt);
    Sval_list(N)=Sval;
end

end

