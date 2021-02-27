function IRB120 = makeIRB120()
    % Define o IRB20
    L(1) = Revolute('d', 0.290, 'a', 0, 'alpha', -pi/2, 'qlim', [-11*pi/12 11*pi/12]);
    L(2) = Revolute('offset', -pi/2, 'd', 0, 'a', 0.270, 'alpha', 0, 'qlim', [-11*pi/18 11*pi/18]);
    L(3) = Revolute('d', 0, 'a', 0.070, 'alpha', -pi/2, 'qlim', [-11*pi/18 7*pi/18]);
    L(4) = Revolute('d', 0.302, 'a', 0, 'alpha', pi/2, 'qlim', [-8*pi/9 8*pi/9]);
    L(5) = Revolute('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-2*pi/3 2*pi/3]);
    L(6) = Revolute('offset', pi, 'd', 0.072, 'a', 0, 'alpha', 0, 'qlim', [-20*pi/9 20*pi/9]);

    % Definição do robô IRB120
    IRB120 = SerialLink(L, 'name', 'IRB120');
end
