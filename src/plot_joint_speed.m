function plot_joint_speed(control_sig)
% plot joint speed over time

figure(4)
hold on
for i = 1:size(control_sig, 1)
    plot(control_sig(i,:))
end
name = legend(compose('$\\dot{q}_{%d}$', 1:size(control_sig, 1)));
set(name,'Interpreter','latex');
xlabel('Iterações')
ylabel('Velocidade/sinal de controle (m/s, rad/s)')

exportgraphics(gca, "C:\Users\gabri\OneDrive\Desktop\4.jpg", 'Resolution',300)
hold off

end
