function r = b2_path(t, wn)
x = 0.20*(sin(wn*t) + sin(4*wn*t)) + 0.428;
y = 0.500;
z = 0.20*(cos(wn*t) + cos(4*wn*t)) + 0.569;

r = [x, y, z]';
end

