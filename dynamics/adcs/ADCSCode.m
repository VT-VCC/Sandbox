function [Rbi]=ADCSCode(year,month,day,hour,minute,second)
%%%%%%%%%%%%%%%%%Inputs%%%%%%%%%%%%%%



%%%%%%%%%%%%%Row vectors?????????????????


%% Converts Time to Julian Date and Julian Date to Sun Inertial Vector
[JD]=Date2Julian(year,month,day,hour,minute,second);
[si,~,~,~]=JD2SunInertial(JD);
si=si./norm(si); %Normalizes Initerial sun vector

%% GPS Output Position ECEF
ECEFx=1234000;%placeholder
ECEFy=1234000;%placeholder
ECEFz=1234000;%placeholder
location=[ECEFx,ECEFy,ECEFy];
%% Converts GPS Positions to Lat Long and Alt
ecoord = LatLong(location);
latitude=ecoord(1);
longitude=ecoord(2);
altitude=ecoord(3);

%% Gets Inertial Magnetic Field Vector
time=[year,month,day,hour,minute,second];
coord='geodetic'; %Will have to determine if this is what we want
%%%%%%%%%%%%%Could Change IGRF to a more simple model %%%%%%%%%%%%%%%
[Bx, By, Bz] = igrfScalar(time, latitude, longitude, altitude, coord);
mi=[Bx;By;Bz];
mi=mi./norm(mi);


%% Input Readings of Body Framm from Sun Sensor and Magnetometer

sb=[-1,1,0.5]'; %Placeholder
mb=[1,1,1]'; %Placeholder

%% Uses Inertial and Body vectors to create rotation matrix from inertial to
%body
Rbi=triad(sb,mb,si,mi); %All inputs need to be column vectors
