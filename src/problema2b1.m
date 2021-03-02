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
K = 0.8;
q = qi;
e = 0;

% Allocate time series
t_step = 0.1;
t = 1:t_step:50;
t0 = 0;

% Pre allocate vectors for plotting
path = zeros(3, length(t));
rpy_path = zeros(3, length(t));
qpath = zeros(IRB120.n, length(t));
control_sig = zeros(IRB120.n, length(t));
err = zeros(6, length(t));
nerr = zeros(length(t));

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
    dxd = xd - xd_ant;
    
    % Get jacobian
    J = IRB120.jacob0(q, 'rpy');
    % Control 
    u = speed_saturation(pinv(J)*(K*e), t_step);
    % Integration of the q' signal
    q = q + 0.1*u;
    
    IRB120.plot(q'); % plot current configuration
    plotp(p, '.c'); % plot path
    
    % store parameters
    control_sig(:,i) = u * t_step;
    path(:, i) = p;
    qpath(:, i) = q';
    rpy_path(:, i) = rpy;
    err(:, i) = e';
    nerr(i) = norm(e);
    
    xd_ant = xd;

end

% plotp(path);
hold off

% plot actuator path and angle path over time
figure(2)
tiledlayout(1, 2)

nexttile
plotp(path, '-r');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

nexttile
hold on
for i = 1:3
    plot(rpy_path(i, :));
end
hold off
legend('row \psi', ' pitch \theta', 'yaw \phi');
xlabel('Iterações');
ylabel('Angulo (rad)');

% plot join configurations x time
figure(3)
hold on
for i = 1:IRB120.n
    plot(qpath(i, :));
end
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6', 'q_7');
xlabel('Iterações');
ylabel('Deslocamento (m, rad)');

% plot joint speed over time
figure(4)
hold on
for i = 1:IRB120.n
    plot(control_sig(i,:))
end
name = legend('$\dot{q}_{1}$', '$\dot{q}_{2}$', '$\dot{q}_{3}$', ...
    '$\dot{q}_{4}$', '$\dot{q}_{5}$', '$\dot{q}_{6}$', '$\dot{q}_{7}$');
set(name,'Interpreter','latex');
xlabel('Iterações')
ylabel('Velocidade/sinal de controle (m/s, rad/s)')
hold off

% plot errors
figure(5)
tiledlayout(1, 2)

nexttile
plot(nerr)
xlabel('Iterações')
ylabel('Norma do erro: |e|')

nexttile
hold on
for i = 1:6
    plot(err(i,:))
end
legend('erro x', 'erro y', 'erro z', 'erro \psi', 'erro \omega', 'erro \phi');
xlabel('Iterações')
ylabel('Erro (m, rad)')
