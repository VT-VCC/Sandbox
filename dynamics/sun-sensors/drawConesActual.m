function drawConesActual(alpha_XZ_deg, alpha_YZ_deg, alpha_XY_deg)
% function for drawing cones and intersection vectors for
% spatially dislocated origin cones produced by input angles of incidence.

% input light angle calculated by XZ- and YZ-bodyplane solar panel current
% (0-90 deg)
alpha_XZ_deg = 55;
alpha_YZ_deg = 25;
alpha_XY_deg = 55;

% generate cone about x-axis
alpha_YZ = alpha_YZ_deg * pi/180;
% cone aperture half-angle
alpha_x = pi/2 - alpha_YZ;
% discrete running vectors for calculating/displaying
x_x = linspace(0.5, 800, 2000);
theta_x = linspace(0, 2*pi, 100);
[x_x, theta_x] = meshgrid(x_x, theta_x);
% calculate y and z based off free variables x and theta about x
z_x = (x_x-0.5).*cos(theta_x)*tan(alpha_x);
y_x = (x_x-0.5).*sin(theta_x)*tan(alpha_x);

% generate cone about y-axis
alpha_XZ = alpha_XZ_deg * pi/180;
alpha_y = pi/2 - alpha_XZ;
y_y = linspace(0.5, 800, 2000);
theta_y = linspace(0, 2*pi, 100);
[y_y, theta_y] = meshgrid(y_y, theta_y);
z_y = (y_y-0.5).*cos(theta_y)*tan(alpha_y);
x_y = (y_y-0.5).*sin(theta_y)*tan(alpha_y);

% generate cone about z-axis
% alpha_XY = alpha_XY_deg * pi/180;
% alpha_z = pi/2 - alpha_XY;
% z_z = linspace(0.5, 800, 2000);
% theta_z = linspace(0, 2*pi, 100);
% [z_z, theta_z] = meshgrid(z_z, theta_z);
% x_z = (z_z-0.5).*cos(theta_z)*tan(alpha_z);
% y_z = (z_z-0.5).*sin(theta_z)*tan(alpha_z);

% calculate intersection theta angles and vectors
Tx = tan(alpha_x);
Ty = tan(alpha_y);
theta_x_isct_1 = asin(sqrt((Tx^2+1)/(Tx^2*(Ty^2+1))));
theta_x_isct_2 = pi - theta_x_isct_1;
v_size = 125;
v_1 = [v_size v_size*sin(theta_x_isct_1)*Tx v_size*cos(theta_x_isct_1)*Tx];
v_2 = [v_size v_size*sin(theta_x_isct_2)*Tx v_size*cos(theta_x_isct_2)*Tx];

% draw
clf
run('cube_driver.m');
mesh(x_x, y_x, z_x, x_x);
mesh(x_y, y_y, z_y, y_y);
quiver3(0, 0, 0, v_1(:,1), v_1(:,2), v_1(:,3), '-k', 'LineWidth', 3);
quiver3(0, 0, 0, v_2(:,1), v_2(:,2), v_2(:,3), '-k', 'Linewidth', 3);

hold off;
end