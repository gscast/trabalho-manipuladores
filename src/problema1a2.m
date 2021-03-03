clc
clear all
close all

L(1) = Revolute('d', .290, 'a', 0, 'alpha', -pi/2,'qlim',(11/12)*[-pi pi] );
L(2) = Revolute('d', 0, 'a', 0.270, 'alpha', 0,'offset',-pi/2,'qlim',(11/18)*[-pi +pi]);
L(3) = Revolute('d', 0, 'a', 0.07, 'alpha', -pi/2,'qlim',([-(11/18)*pi +(7/18)*pi]));
L(4) = Revolute('d', 0.302, 'a', 0, 'alpha', pi/2,'qlim',(8/9)*[-pi +pi]);
L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2,'qlim',(2/3)*[-pi +pi]);
L(6) = Revolute('d', 0.072, 'a', 0, 'alpha', 0,'offset',pi,'qlim',(20/9)*[-pi +pi]);

irb120 = SerialLink(L, 'name', 'IRB120');

q = [0 0 0 0 -pi/2 0]'; % Define configuração inicial do robô

pd = [.380 .380 .500];
rd =SO3().R();
td = SE3(rd,pd);

rpy_i = tr2rpy(td,'xyz');

K = 1; % Define ganho
epsilon = 10e-5; % Define critério de parada
e_ant = 1;
e = 0; 

i = 0;

figure(1)
irb120.plot(q'); % Plot robô na configuração inicial
hold on
td.plot('rgb') % Plot pose desejada

qpath = zeros(6,60);

while (norm(e - e_ant) > epsilon) % Critério de parada
    i = i+1; % contador
    J = irb120.jacob0(q,'rpy'); %Jacobiana Analitica, é necessaria ser ela
    T = irb120.fkine(q); % Cinemática direta para pegar a pose do efetuador 
    p = transl(T); % Translação do efetuador
    R = SO3(T); 
    R = R.R(); % Extrai rotação do efetuador
    rpy = tr2rpy(R,'xyz');
    p_err = pd - p; % Erro de translação
    
    rpy_t = rpy_i - rpy; % Erro de rotação 
    
    e_ant = e;
    e = [p_err'; rpy_t']; % Vetor de erro
    
    u = inv(J)*K*e; % Lei de controle

    q = q + 0.1*u; % Cálculo de q (Regra do trapézio)
    
    irb120.plot(q');
    control_sig(:,i) = u; % Sinal de controle
    err(i) = norm(e); % Norma do erro
    norm(e)
    
    %erros
    e_x(i)=e(1,:);
    e_y(i)=e(2,:);
    e_z(i)=e(3,:);
    e_R(i)=e(4,:);
    e_P(i)=e(5,:);
    e_Y(i)=e(6,:);
      
    path(:, i) = p;
    rpy_path(:, i) = rpy;
    qpath(:,i)=q';
    
    %trajetoria real-time
    plotp(p','.c');
    
end
hold off

%% Plot sinal de controle/velocidade e norma do erro
plot_joint_config(qpath)

figure(2)

hold on
subplot(3,3,1)
plot(180*control_sig(1,:))
title('Velocidade q1')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

subplot(3,3,2)
plot(180*control_sig(2,:))
title('Velocidade q2')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

subplot(3,3,3)
plot(180*control_sig(3,:))
title('Velocidade q3')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

subplot(3,3,4)
plot(180*control_sig(4,:))
title('Velocidade q4')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

subplot(3,3,5)
plot(180*control_sig(5,:))
title('Velocidade q5')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

subplot(3,3,6)
plot(180*control_sig(6,:))
title('Velocidade q6')
xlabel('Iterações')
ylabel('velocidade:u(º/s)')

hold off

%Plot Norma dos Erros
figure(5)
hold on
plot(err)
xlabel('Iterações')
ylabel('Norma do erro: |e|')
hold off

%Plot relaciona a Erros
figure(4)
hold on
subplot(3,3,1)
plot(e_x)
title('Erro: x')
xlabel('Iterações')
ylabel('Erro em x (m)')

subplot(3,3,2)
plot(e_y)
title('Erro: y')
xlabel('Iterações')
ylabel('Erro em y (m)')

subplot(3,3,3)
plot(e_z)
title('Erro: z')
xlabel('Iterações')
ylabel('Erro em z (m)')

subplot(3,3,4)
plot(e_R)
title('Erro: roll')
xlabel('Iterações')
ylabel('Erro em roll (rad)')

subplot(3,3,5)
plot(e_P)
title('Erro: Pitch')
xlabel('Iterações')
ylabel('Erro em Pitch (rad)')

subplot(3,3,6)
plot(e_Y)
title('Erro: Yaw')
xlabel('Iterações')
ylabel('Erro em Yaw (rad)')

hold off

%Plot relaciona aos Deslocamentos
figure(3)
hold on
for i = 1:6
    plot(180*qpath(i, :));
end
legend('q_1', 'q_2', 'q_3', 'q_4', 'q_5', 'q_6');
xlabel('Iterações');
ylabel('Angulo (º)');
hold off

%Trajetorias
figure(6)
hold on
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
legend('roll \psi', ' pitch \theta', 'yaw \phi');
xlabel('Iterações');
ylabel('Angulo (rad)');
hold off