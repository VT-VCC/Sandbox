%% sun_body function
% This function determines the sun vector in the body frame of the cubesat
% from the current measurements of its five solar cells
%% parameters
% i1 - front face in body frame, +x direction
% i2 - left face in body frame, +y direction
% i3 - rear face in body frame, -x direction
% i4 - right face in body frame, -y direction
% i5 - bottom face in body frame, -z direction
% maxA - maximum current of solar panel, for normalization
% There is no solar panel on the top face of the cubesat.

function [s_b] = sun_body( i1, i2, i3, i4, i5, maxA )

normalized = [i1 i2 i3 i4 i5] / maxA; % normalize currents to max of 1
five_activated = 0; % flag false if no i5 current

if sum(normalized) <= 1 % if one or fewer solar panels are detecting light
    s_b = [0 0 0]; % cannot calculate sun vector
elseif sum(normalized) > 4 % if 4 or more solar panels are detecting light
    s_b = [0 0 0]; % cannot calculate sun vector
elseif i1 ~= 0 && i3 ~= 0 % if front and back panels are detecting light
    s_b = [0 0 0]; % irreconcilable?
elseif i2 ~= 0 && i4 ~= 0 % if right and left panels are detecting light
    s_b = [0 0 0]; % irreconcilable?
else
    % remaining cases:
    % [1, 2]
    % [1, 2, 5]
    % [1, 4]
    % [1, 4, 5]
    % [2, 3]
    % [2, 3, 5]
    % [3, 4]
    % [3, 4, 5]
    five_activated = ceil(i5); % if i5 has > 0 current, flips flag to 1
    
        
    
end
    
end