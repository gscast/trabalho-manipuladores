function w = joint_limit(q, qlim)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
mean_qlim = geomean(mod(qlim, 2*pi), 2);
w = -1/(2*length(q))*sum(((q - mean_qlim)./abs(diff(qlim, 1, 2))).^2);
end

