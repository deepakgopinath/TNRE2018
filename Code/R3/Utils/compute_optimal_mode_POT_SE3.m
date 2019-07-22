function [ best_mode ] = compute_optimal_mode_POT_SE3(xg_T, xr_T_true)
    global cm nd xr_T;
    Pk = zeros(nd, 1);
    Pcm = zeros(length(cm), 1); %Information density for each mode. 
    for i=1:nd-3
        Pk(i) = abs((xg_T(i, 4) - xr_T(i, 4))/(xg_T(i, 4) - xr_T_true(i, 4)));
    end
    Rg = xg_T(1:3, 1:3); Rr = xr_T(1:3, 1:3); Rr_true = xr_T_true(1:3, 1:3);
    Rdiff_w = Rg*(Rr^-1); Rdiff_w_true = Rg*(Rr_true^-1);
    
    %THE FOLLOWING IS SUSPICIOUS. WATCH OUT!!!
    [w,~] = AxisAng3(MatrixLog3(Rdiff_w));
    [w_true, ~] = AxisAng3(MatrixLog3(Rdiff_w_true));
    Pk(4:end) = w./w_true;
    for i=1:length(cm)
        Pcm(i) = sum(Pk(cm{i}));
    end
    %decide the fractional potential for the rotational dimensions. 
    best_mode = compute_best_mode(Pcm);
end