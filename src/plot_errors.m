function plot_errors(err)
% plot errors

figure(5);
nerr = vecnorm(err);
t = tiledlayout(1, 2);

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

exportgraphics(t, "C:\Users\gabri\OneDrive\Desktop\5.jpg",'Resolution',300)
hold off

end

