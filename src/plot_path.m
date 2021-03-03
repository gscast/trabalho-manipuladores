function plot_path(path, rpy_path)
% plot actuator path and angle path over time
figure(2)
t = tiledlayout(1, 2);

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
legend('roll \psi', ' pitch \theta', 'yaw \phi');
xlabel('Iterações');
ylabel('Angulo (rad)');

% exportgraphics(t, "C:\Users\gabri\OneDrive\Desktop\2.jpg", 'Resolution',300)
hold off

end

