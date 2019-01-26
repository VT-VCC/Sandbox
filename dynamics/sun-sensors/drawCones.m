function drawCones(alpha_XZ_deg, alpha_YZ_deg)
% function for drawing cones and intersection vectors for
% coinciding origin cones produced by input angles of incidence

% input light angle calculated by XZ- and YZ- bodyplane solar
% panel current (0-90 deg)

alpha_XZ_deg = 55;
alpha_YZ_deg = 25;

% generate cone about x-axis
alpha_YZ = alpha_YZ_deg * pi/180;
% cone aperture half angle
alpha_x = pi/2 - alpha_YZ;
% discrete running vectors for calculating/displaying
x_x = linspace(0, 10, 25);
theta_x = linspace(0, 2*pi, 100);
[x_x, theta_x] = meshgrid(x_x, theta_x);
% calculate z and y based off free variables x and theta about x
z_x = x_x .* cos(theta_x) * tan(alpha_x);
y_x = x_x .* sin(theta_x) * tan(alpha_x);

% generate cone about y-axis
alpha_XZ = alpha_XZ_deg * pi/180;
alpha_y = pi/2 - alpha_XZ;
y_y = linspace(0, 10, 25);
theta_y = linspace(0, 2*pi, 100);
[y_y, theta_y] = meshgrid(y_y, theta_y);
z_y = y_y .* cos(theta_y) * tan(alpha_y);
x_y = y_y .* sin(theta_y) * tan(alpha_y);

% calculate intersection theta angles and vectors

Tx = tan(alpha_x);
Ty = tan(alpha_y);
theta_x_isct_1 = asin(sqrt((Tx^2+1)/(Tx^2*(Ty^2+1))));
theta_x_isct_2 = pi - theta_x_isct_1;

v_size = 15;
v_1 = [v_size v_size*sin(theta_x_isct_1)*Tx v_size*cos(theta_x_isct_1)*Tx];
v_2 = [v_size v_size*sin(theta_x_isct_2)*Tx v_size*cos(theta_x_isct_2)*Tx];

% draw
clf;
% set up figure and draw satellite
run('cube_driver.m');
% draw cone 1, about x-axis
mesh(x_x, y_x, z_x);
% draw cone 2, about y-axis
mesh(x_y, y_y, z_y);
% draw intersection vectors
quiver3(0, 0, 0, v_1(:,1), v_1(:,2), v_1(:,3), '-b', 'LineWidth', 3);
quiver3(0, 0, 0, v_2(:,1), v_2(:,2), v_2(:,3), '-b', 'LineWidth', 3);
hold off
end
