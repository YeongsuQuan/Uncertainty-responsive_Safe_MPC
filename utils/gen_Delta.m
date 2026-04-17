function [Sval_list] = gen_Delta(t_acc,ts,v_bar,a_bar_low,a_bar_high,xt,M)

for N = 1:M
    [Sval] = gen_Sval(t_acc,ts,v_bar,a_bar_low,a_bar_high,N,xt);
    Sval_list(N)=Sval;
end

end

