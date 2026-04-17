function [x_obs,y_obs]=gen_pedestrian(xc,yc,theta)

t = linspace(0, 2*pi, 200);
r1 = 0.25;
r2 = 0.15;
xt = r1 * cos(t) + xc;
yt = r2 * sin(t) - yc;
cot = cos(theta); sit = sin(theta);
x_obs = xt * cot - yt * sit;
y_obs = xt * sit - yt * cot;

end

