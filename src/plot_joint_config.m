function plot_joint_config(qpath)
% plot join configurations x time
figure(3)
hold on
for i = 1:size(qpath, 1)
    plot(qpath(i, :));
end
legend(compose('q_%d', 1:size(qpath, 1)));
xlabel('Iterações');
ylabel('Deslocamento (m, rad)');
hold off
end
