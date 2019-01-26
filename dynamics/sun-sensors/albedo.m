function Albicss = albedo ( t, ncss, block, vb, rb, sb )
%% assume
% neglect albedo due to specular reflectance
% circular orbit
% perfectly spherical earth
% conical field of view for coarse sun sensors
%% constants
% alpha_max: S/C field of view half angle
% amax: solid angle subtended by the portion of the earth's surface
%       visible from the spacecraft (steradian)
% dA: solid angle subtended by dAe, a small piece of sunlit earth, from 
%     the spacecraft's field of view (steradian)
% re: radius of earth (km)
% rmag: distance from the center of the earth to S/C in circular orbit
%% User sets these constants in the ALBEDO *subroutine*
% alb: albedo constant; ratio of diffuse reflectance to incident light
% altp: altitude of the spacecraft at perigee (km)
% blockage: flag to indicate whether the CSS are blocked [0:n, 1:y]
%           If yes, the blockage numbers must be in the (P,4) array BLOCK
% cssimax: maximum current output by CSS to ACS when sun lies along CSS
%          boresight (include any scaling by attitude control electronics)
% csslm: cosine of half angle of the CSS FOV, assumes all of the P CSS have
%        the same size conical FOV
% n: number of concentric circles
% na: number of equal areas
% P: number of coarse sun sensors
%% User passes in these constants from the calling routing, each time step
% Ncss: unit vectors of CSS boresight in S/C body coordinates
%% User passes in these variables at each time step
% block: a Px4 array which holds the blockage values for the CSS modeled as
%        a blockage cone. First three element of each row are the S/C body
%        coordinates in the cone axis. The fourth element is the angle of
%        the blockage cone in degrees
% rb: spacecraft position unit vector from the earth center to S/C in S/C
%     body coordinates
% sb: unit vector from the earth to the sun in the S/C body coordinates
% t: time
% vb: spacecraft velocity unit vector in S/C body coordinates
%% program variables
% A: 3x3 transformation matrix from S/C body coordinates to 1-2-3 frame
% a1: area of central subsegment of unit S/C centered sphere, defined as
%     one unit area equal to the size of dA
% blocked: test flag, if the Lth CSS blockage blots out the albedo light
%          from the Kth infinitesimal area, blocked = 0. If not blocked,
%          blocked = 1
% cgamma and sgamma: cosine and sine, respectively of the angle from axis
%                    of FOV cone to normal of Kth dAe (deg)
% cssimax: maximum possible CSS output
% D: distance from Kth dAe to the spacecraft position
% di: CSS output current due to albedo from the Kth dAe
% half_theta: equals theta/2
% ho: unit vector in direction of orbit normal: unit vector for axis 2
% i: refers to the ith concentric circle out of n concentric circles
% j: refers to the jth radial area in the ith annulus
% k: refers to the kth unit area out of na unit areas
% kappa: reflective constant for kth dA
% l: refers to the lth css out of p css
% lit: flag , indicates if kth dA is in light or darkness
% m: number of radial divisions in the ith circle
% ndotu: css head unit vector dotted with kth sc to ground unit vector to
%        determine if light from the kth dAe is visible to the spacecraft
% Norm: outward pointing normal vector for kth dAe
% one: ho x rb, unit vector along axis 1
% ri: distance from spacecraft FOV cone's axis to area centers for ith 
%     annulua
% s123: sun unit vector in 1-2-3 frame coordinates
% theta: size of radial divisions in the ith circle (deg)
% U: unit vector from s/c to center of the kth dAe (location of kth dA's
%    norm) in 1-2-3 coordinates
% u123: temporary holding vector for each u
% usc: u transformed from 1-2-3 coordinates from to s/c body coordinates
%% output
% albicss: finall css currents due to albedo
%%
% function abicss = albedo ( t, ncss, block, vb, rb, sb )

U = zeros(1000, 3);
Norm = zeros(1000, 3);
D = zeros(1000, 1);
Albicss = zeros(10, 1);
A = zeros(3, 3);
U123 = zeros(3, 1);

% begin initialization loop
if t == 0
    
    % set constants
    blockage = 1.0;
    n = 6;
    na = (2 * n - 1) ^ 2;
    p = 8; % 5;
    re = 6378.0;
    csslm = cosd(80.0); % FOV half angle
    altp = 350.0; % guess orbital parameters
    rmag = re + altp;
    alpha_max = asind(re / rmag);
    amax = 2 * pi * (1 - cosd(alpha_max));
    a1 = amax / na;
    alb = 0.30; % assumed earth albedo constant
    cssimax = 1.0; % depends on css specs
    kappa = alb * cssimax * a1/pi;
    
    % calculate unit vectors from spacecraft to kth area normal
    k = 1;
    for i = 1:n
        m = 8 * (i - 1);
        if m == 0
            U(k, 1) = 0;
            U(k, 2) = 0;
            U(k, 3) = 0;
            k = k + 1;
        else
            theta = 360.0 / m;
            half_theta = 0.5 * theta;
            ri = sqrt(4 * (i - 1) * (i - 1) + 1);
            cgamma = (na - ri^2 + ri^2 * (cosd(alpha_max)))/na;
            sgamma = sqrt(1 - cgamma^2);
            for j = 1:m
                U(k, 1) = sgamma * cosd(theta * j - half_theta);
                U(k, 2) = sgamma * sind(theta * j - half_theta);
                U(k, 3) = -cgamma;
                k = k + 1;
            end
        end
    end
    
    % calcularte area normals
    c = rmag^2 - re^2;
    for k = 1:na
        b = rmag * U(k, 3);
        D(k) = -b - sqrt(b^2 - c);
        Norm(k, 1) = D(k) * U(k, 1) / re;
        Norm(k, 2) = D(k) * U(k, 2) / re;
        Norm(k, 3) = (rmag + D(k) * U(k, 3)) / re;
    end
end
% end initialization loop

% at all times,
% find matrix A to go from s/c body to orbital frame 1-2-3
ho = crossu(rb, vb);
one = crossu(ho, rb);
for kl = 1:3
    A(1, kl) = one(kl);
    A(2, kl) = ho(kl);
    A(3, kl) = rb(kl);
end

% transform sun unit vector from s/c body to 1-2-3
S123 = to123(A, sb);

% find current in lth css due to albedo from lit areas
for l = 1:p
    Albicss(l) = 0;
    for k = 1:na
        
        % determine if the kth area is lit
        sdotn = (S123(1) * Norm(k, 1) + S123(2) * Norm(k, 2) + S123(3)...
            * Norm(k, 3));
        if sdotn > 0
            
            % determine if the kth dA is the lth css's FOV
            for kl = 1:3
                U123(kl) = U(k, kl);
            end
            usc = tosc(A, U123);
        end
        ndotu = (ncss(1, l) * usc(1) + ncss(2, l) * usc(2) + ...
            ncss(3, l) * usc(3));
        if ndotu > csslm
            
            % if there is css blockage, determine if it blocks
            % the albedo from the kth area
            blocked = 1;
            if blockage == 1
                udotblock = block(l, 1) * usc(1) + block(l, 2) * ...
                    usc(2) + block(l, 3) * usc(3);
                if udotblock > cosd(bloakc(l, 4))
                    blocked = 0;
                end
                di = kappa * ndotu * sdotn * blocked;
                Albicss(l) = Albicss(l) + di;
            end
        end
    end
end  
end
