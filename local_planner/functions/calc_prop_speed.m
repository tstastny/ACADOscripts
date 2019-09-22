function n_p = calc_prop_speed(u_T,v,aoa,model_params,sysid_config)

for i = 1:length(model_params)
    eval([model_params(i).Name,' = ',num2str(model_params(i).Value),';']);
end

vmin = 10;                                                                  % minimum stabilizable airspeed on trim map
vmax = 25;                                                                  % maximum stabilizable airspeed on trim map
n_T0_vmin = (-cT_1*vmin/sysid_config.d_prop + ...                           % zero-thrust prop speed at minimum airspeed
    sqrt((cT_1*vmin/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;
n_T0_vmax = (-cT_1*vmax/sysid_config.d_prop + ...                           % zero-thrust prop speed at maximum airspeed
    sqrt((cT_1*vmax/sysid_config.d_prop)^2 - cT_0*4))/2/cT_0;
vp =  v .* cos(aoa - sysid_config.epsilon_T);                       % inflow at propeller

sig_vp = (vp - vmin)./(vmax - vmin);                                        % prop inflow linear interpolater
n_p = ...
    (n_T0_vmin + u_T .* (sysid_config.rpm_max/60 - n_T0_vmin)) .* (1 - sig_vp) + ...          
    (n_T0_vmax + u_T .* (sysid_config.rpm_max/60 - n_T0_vmax)) .* sig_vp;  