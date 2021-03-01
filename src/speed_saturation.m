function u_sat = speed_saturation(u, t_step)
    umax = t_step*[0.500 25*pi/18 25*pi/18 25*pi/18 16*pi/9 16*pi/9 7*pi/3]';
    umin = -umax;
    
    u_sat = min(u, umax);
    u_sat = max(u_sat, umin);
end

