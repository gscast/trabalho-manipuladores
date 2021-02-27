function trail = make_trail()
    % Define o trilho
    L = Link('theta', 0, 'a', 0, 'alpha', pi/2, 'qlim', [0 0.5]);
    trail = SerialLink(L, 'base',  trotx(-pi/2, 'rad'), 'name', 'trail');
end
    

