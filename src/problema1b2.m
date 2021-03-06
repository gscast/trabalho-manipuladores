close all
clear all
clc

%% Criando o robô ABB IRB 120 - 6 graus de liberdade 

% Declaração das juntas baseados no modelo da tabeela DH
L(1) = Revolute('d',.290,'alpha',-pi/2,'qlim',(11/12)*[-pi pi]);
L(2) = Revolute('a',.270,'offset',-pi/2, 'qlim',(11/18)*[-pi pi]);
L(3) = Revolute('a',.070,'alpha',-pi/2,'qlim',([-(11/18)*pi (7/18)*pi]));
L(4) = Revolute('d',.302,'alpha', pi/2, 'qlim', (8/9)*[-pi pi]);
L(5) = Revolute('alpha',-pi/2,'qlim',(2/3)*[-pi pi]);
L(6) = Revolute('d', .072, 'offset', pi, 'qlim', (20/9)*[-pi pi]);

i120 =  SerialLink(L, 'name', 'IRB 120 - GGW');

%% Item b) implementando um controle cinemático para as seguitnes trajetórias:

q = [0 0 0 0 -pi/2 0]'; % posição inicial
i120.plot(q'); % observando a posição do robô
 
%% ii) Circuferência y = .200 - Posição

p_centro = [.428 .200 .669]';
wn = pi/10; % velocidade de rotação

raio = .050; % raio de rotação
x0 = p_centro(1) + raio*cos(0); % posicionando o efetuador na extremidade da circunferência no plano xz inercial
y0 = p_centro(2);
z0 = p_centro(3) + raio*sin(0); % mesmo para z0 

K = 1; % ganho para ação de controle;
t = tic;
e = inf(6,1);
epsilon = 10e-5;

% Allocate time series
t_step = 0.1;
t1 = 0:t_step:30;

% Pre allocate vectors for plotting
path = zeros(3, length(t1));
rpy_path = zeros(3, length(t1));
qpath = zeros(i120.n, length(t1));
control_sig = zeros(i120.n, length(t1));
err = zeros(6, length(t1));
nerr = zeros(length(t1));
i=0;

pd=[x0 y0 z0]; % posiciona na extremidade da circ

Rd2 = SO3.Rz(0).Ry(0).Rx(0);
Rd2 = Rd2.R();
Td2 = SE3(Rd2,pd);
rpy = tr2rpy(Td2,'xyz');
hold on
Td2.plot('rgb'); % extremidade da circuferência 

while (norm(e)>epsilon)
    i=i+1;
    T = i120.fkine(q);
    Jc = i120.jacob0(q,'rpy');
    J = Jc(1:3,:);
    R = SO3(T); 
    R = R.R(); % Extrai rotação do efetuador
    rpy_atual = tr2rpy(R,'xyz');
    rpy_aplicado = rpy - rpy_atual;
    p = transl(T);
    p_til = pd - p;
    
    e = [p_til'];
    u = pinv(J).* K * e;
    dt = toc(t);
    t=tic;
    u = speed_saturation(u,dt);
    q = q + u*dt;
    te=i120.fkine(q);
    i120.plot(q')
    plotp([p(1) p(2) p(3)]', '.r');
    control_sig(:,i) = u * dt;
    path(:, i) = p;
    err(:, i) = [e' rpy_aplicado];
    qpath(:, i) = q';
    rpy_path(:, i) = rpy_atual;
    index = i;
end

t2 = 0:0.1:30; 
x = (raio*cos(wn*t2)+p_centro(1));
z = (raio*sin(wn*t2)+p_centro(3));

temp = transl(te); % pega a atual posição do efetuador

% q = [temp(1) temp(2) temp(3) rpy(1) rpy(2) rpy(3)]';

xd_ant=[temp]';

t=tic;

pd=[x0 y0 z0]; % posiciona na extremidade da circ

Rd2 = SO3().R();
Td2 = SE3(Rd2,pd);

rpy = tr2rpy(Td2,'xyz');

path=zeros(3,length(t));

for i=1:length(t2)
    
    if i==1
        T=te;
    else    
        T = i120.fkine(q); % T. Homogênea Cinemática Direta
    end
    
    Jc = i120.jacob0(q,'rpy'); % Obtem a jacobiana analítica
    J = Jc(1:3,:);    %  Reduz a jacobiana para 3x6
    p = transl(T); % Obtem o ponto atual; 
    R = SO3(T); 
    R = R.R(); % Extrai rotação do efetuador
    rpy_atual = tr2rpy(R,'xyz');   
    if ((i+1)<=length(t2))
        pd = [x(i+1) y0 z(i+1)];
    else
        pd = [x(i) y0 z(i)];
    end
    rpy_aplicado = rpy - rpy_atual;
    p_til = pd - p; 
%     e = [p_til'];
    
    xd = [pd]';  
    xv = [p]';
    
    e = xd - xv;
    dx = xd - xd_ant;
    
    xd_ant = xd;
    
    u = pinv(J)*(dx + (K * e));
    u = speed_saturation(u,0.1);
    q = q + u*0.1;
    i120.plot(q')
    plotp([p(1) p(2)+0.2 p(3)]', '.r');
    control_sig(:,index) = u * t_step;
    path(:, index) = p;
    qpath(:, index) = q';
    rpy_path(:, index) = rpy_atual;
    err(:, index) = [e' rpy_aplicado];
    index=index+1;
end

hold off

%% Gráficos

% plot actuator path and angle path over time
figure;
hold on 
subplot(2,2,1), plotp(path, '-r');
grid on
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
subplot(2,2,2), plot(180*rpy_path(1, :));
grid on
legend('roll \psi');
xlabel('Iterações');
ylabel('Angulo (º)');
subplot(2,2,3), plot(180*rpy_path(2,:));
grid on
legend('pitch \theta');
xlabel('Iterações');
ylabel('Angulo (º)');
subplot(2,2,4),plot(180*rpy_path(3,:));
grid on
legend('yaw \phi');
xlabel('Iterações');
ylabel('Angulo (º)');
hold off

% plot join configurations x time
figure;
hold on
subplot(2,3,1),plot(180*qpath(1, :));
grid on
legend('q_1');
xlabel('Iterações');
ylabel('Deslocamento (º)');
subplot(2,3,2),plot(180*qpath(2, :));
grid on
legend('q_2');
xlabel('Iterações');
ylabel('Deslocamento (º)');
subplot(2,3,3),plot(180*qpath(3, :));
grid on
legend('q_3');
xlabel('Iterações');
ylabel('Deslocamento (º)');
subplot(2,3,4),plot(180*qpath(4, :));
grid on
legend('q_4');
xlabel('Iterações');
ylabel('Deslocamento (º)');
subplot(2,3,5),plot(180*qpath(5, :));
grid on
legend('q_5');
xlabel('Iterações');
ylabel('Deslocamento (º)');
subplot(2,3,6),plot(180*qpath(6, :));
grid on
legend('q_6');
xlabel('Iterações');
ylabel('Deslocamento (º)');
hold off

% plot joint speed over time

figure;
hold on
subplot(2,3,1),plot(180*control_sig(1,:));
grid on
legend('{q}_{1}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
subplot(2,3,2),plot(180*control_sig(2,:));
grid on
legend('{q}_{2}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
subplot(2,3,3),plot(180*control_sig(3,:));
grid on
legend('{q}_{3}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
subplot(2,3,4),plot(180*control_sig(4,:));
grid on
legend('{q}_{4}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
subplot(2,3,5),plot(180*control_sig(5,:));
grid on
legend('{q}_{5}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
subplot(2,3,6),plot(180*control_sig(6,:));
grid on
legend('{q}_{6}');
xlabel('Iterações');
ylabel('Velocidade/sinal de controle (graus/s)');
hold off


% plot errors
figure;
nerr = vecnorm(err);
plot(nerr)
grid
xlabel('Iterações')
ylabel('Norma do erro: |e|')

figure;
hold on
subplot(2,3,1),plot(err(1,:));
grid on
legend('erro x');
xlabel('Iterações');
ylabel('Erro (m)');
subplot(2,3,2),plot(err(2,:));
grid on
legend('erro y');
xlabel('Iterações');
ylabel('Erro (m)');
subplot(2,3,3),plot(err(3,:));
grid on
legend('erro z');
xlabel('Iterações');
ylabel('Erro (m)');
subplot(2,3,4),plot(180*err(4,:));
grid on
legend('erro \psi');
xlabel('Iterações');
ylabel('Erro (º)');
subplot(2,3,5),plot(180*err(5,:));
grid on
legend('erro \theta');
xlabel('Iterações');
ylabel('Erro (º)');
subplot(2,3,6),plot(180*err(6,:));
grid on
legend('erro \phi');
xlabel('Iterações');
ylabel('Erro (º)');
hold off