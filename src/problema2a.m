clear
clc
close all;

% Join trail with IRB120
IRB120 = make_trail() * makeIRB120();
q = [0 0 0 0 0 -pi/2 0]';

% Get desired pose
xd = [0.380 0.580 0.600 0 0 0]';
pd = xd(1:3);
Rd = rpy2r(xd(4:end)');
Td = rt2tr(Rd, pd);

% plot the IRB120 in its initial configuration
% and the desired pose
figure(1)
IRB120.plot(q');
hold on
trplot(Td, 'rgb');

% Initialize control parameters
K = 1;
e = 0;

% Allocate time series
dt = 0.1;
t = 1:dt:20;

% Pre allocate vectors for plotting
path = zeros(3, length(t));
rpy_path = zeros(3, length(t));
qpath = zeros(IRB120.n, length(t));
control_sig = zeros(IRB120.n, length(t));
err = zeros(6, length(t));

for i = 1:length(t)
    % get rotation and translation from current join configuration
    % using forward kinematics
    T = IRB120.fkine(q);
    [R, p] = tr2rt(T);
    
    % calculate position error
    p_err = pd - p;
    
    % convert rotation matrix to row-pitch-yaw configuration
    % and calculate orientation error
    rpy = tr2rpy(R);
    rpy_err = (tr2rpy(Rd) - rpy)';
    e = [p_err; rpy_err];
    
    % Control implementation
    J = IRB120.jacob0(q, 'rpy');
    u = pinv(J)*(K*e);
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
    
end
hold off

plot_path(path, rpy_path);
plot_joint_config(qpath);
plot_joint_speed(control_sig);
plot_errors(err);
