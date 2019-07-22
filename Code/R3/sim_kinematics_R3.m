function xr_new = sim_kinematics_R3(xr, uh) %all args are considered to be column vectors. 
    global delta_t;    
    xr_new = xr + uh.*delta_t + normrnd(0, 0.005, length(xr), 1); %add stochasticity to the dynamics. 
end