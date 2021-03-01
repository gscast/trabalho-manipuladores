function r = circle_xz(R, c, t, wn)
x = c(1) + R*cos(wn*t);
y = c(2);
z = c(3) + R*sin(wn*t);

r = [x, y, z]';
end
