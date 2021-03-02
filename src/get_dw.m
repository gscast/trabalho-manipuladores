function dw = get_dw(q, qlim)

dq = 0.01;
dw = zeros(1, length(q))';

prev_q = q;
prev_w = joint_limit(prev_q, qlim);

for i= 1:length(q)
    q = prev_q;
    q(i) = q(i) + dq;
    w = joint_limit(q, qlim);
    dw(i) = (w - prev_w)/dq;
end
    
end

