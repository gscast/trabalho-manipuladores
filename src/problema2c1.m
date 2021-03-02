clear
clc
close all;

% Join trail with IRB120
IRB120 = make_trail() * makeIRB120();
qi = [0 0 0 0 0 -pi/2 0]';

% path params
pc = [0.428, 0.500, 0.569]';
radius = 0.050;
wn = pi/10;

% Desired pose params
pd = circle_xz(radius, pc, 0, wn);
rpy_d = [0 0 0]';
xd_ant = [pd; rpy_d];

% plot the IRB120 in its initial configuration
% and the desired pose
figure(1)
IRB120.plot(qi');
hold on

% Initialize control parameters
K = 0.6;
K_0 = 0.6;
q = qi;
e = 0;

% Allocate time series
dt = 0.1;
t = 1:dt:50;
t0 = 0;

% Pre allocate vectors for plotting
path = zeros(3, length(t));
rpy_path = zeros(3, length(t));
qpath = zeros(IRB120.n, length(t));
control_sig = zeros(IRB120.n, length(t));
err = zeros(6, length(t));

for i = 1:length(t)
        
    % get homogeneous transform from current join configuration
    % using forward kinematics
    T = IRB120.fkine(q);
    % extract rotation matrix and translation vector
    [R, p] = tr2rt(T);
    
    % actuator in path
    if t0
        pd = circle_xz(radius, pc, t(i) - t0, wn);
    %actuator has not reached path point
    elseif norm(pd - p) <= 1e-4
        t0 = t(i);
    end
    
    % calculate position error
    p_err = pd - p;
    
    % convert rotation matrix to row-pitch-yaw configuration
    rpy = tr2rpy(R)';
    % calculate rotation error and assemble error vector
    rpy_err = rpy_d - rpy;
    e = [p_err; rpy_err];
    
    xd = [pd; rpy_d];
    dxd = (xd - xd_ant)/dt;
    
    % Get jacobian
    J = IRB120.jacob0(q, 'rpy');
    % Control 
    dw = get_dw(q, IRB120.qlim);
    kine_cntrl = (eye(IRB120.n) - pinv(J)*J)*K_0*dw;
    u = pinv(J)*(dxd + K*e) + kine_cntrl;
    u = speed_saturation(u, dt);
    % Integration of the q' signal
    q = q + 0.1*u;
    
    IRB120.plot(q'); % plot current configuration
    plotp(p, '.c'); % plot path
    
    % store parameters
    control_sig(:,i) = u * dt;
    path(:, i) = p;
    qpath(:, i) = q';
    rpy_path(:, i) = rpy;
    err(:, i) = e';
    
    xd_ant = xd;

end

hold off

plot_path(path, rpy_path);
plot_joint_config(qpath);
plot_joint_speed(control_sig);
plot_errors(err);
