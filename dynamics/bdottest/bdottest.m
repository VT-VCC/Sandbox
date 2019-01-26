function bdottest

%% Bdot Control

%% Variables and Parameters


m = 1.2; %kg
l = 0.1; %m, placeholder
h = 0.1; %m, placeholder
w = 0.1; %m, placeholder

% I = (1/12)*m*[h^2+w^2; l^2+h^2; h^2+l^2]
I = [0.002; 0.002; 0.002;];

N = 56; %number of torquer loops, pulled from OPAS so placeholder?
A = .003; %average enclosed area of torquer loops, pulled from OPAS so placeholder?
NA = N*A; %total sum of enclosed torquer areas
k = 0.3;
R = 0.01;

K = inv(diag([N*A/R N*A/R N*A/R], 0))
tolerance = 1E-2; %placeholder


ts = 0.1; %placeholder
t = ts;

omega = 1;%placeholder, will be initial Gyroscope reading

go = true;

%% testing
time = 7.3705e+05;
lat = 37.2296;
long = -80.4139;
alt = 408;

Bnot = diag(igrf(time - 15, lat - 1, long - 1, alt), 0)*10E-9; %random placeholder, will be mag reading instead
b1 = Bnot;

while go == true
    readMag = diag(igrf(time, lat, long, alt), 0)*10E-9;
    B = readMag; %read magnetometer
    b2 = B;
    bdot = (b2-b1)/ts;
    M = -k*bdot
    
    V = K*M %voltage?? or current, matrix/vector format
    
    
    t = t+ts;
    
    b1 = b2;
    
    time = time + 15;
    lat = lat + 1;
    long = long + 1;

    omega = omega - 0.1; %placeholder to insure loop exits, will be updated gyro reading maybe
    %% Error Check
    if omega <= tolerance
        go = false;
    end
end
end