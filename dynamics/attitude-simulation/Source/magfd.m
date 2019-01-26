function J=magfd(DATE,ITYPE,ALT,COLAT,ELONG)
% MAGFD
% Function to compute Earths magnetic field
% and components: X,Y,Z,T for a given latitude
% and longitude, date and altitude.
% Uses MATLAB MAT files sh1900.mat to sh2005.mat in 5 yr
% intervals.
%
% Usage: out=magfd(DATE,ITYPE,ALT,COLAT,ELONG);
%
% DATE = date of survey (decimal years)
% ITYPE=1 for geodetic to geocentric (USE 1)
% ALT = altitude of survey relative to sealevel (km +ve up)
% COLAT=90-latitude (decimal degrees)
% ELONG=longitude of survey (decimal degrees)
%
% Output array out contains components X,Y,Z,T in nanoteslas
% X north component
% Y east component
% Z vertical component +ve down
% T total field magnitude
%
% ref: IAGA, Division V, Working Group 8,
% International geomagnetic reference field, 1995
% revision, Geophys. J. Int, 125, 318-321, 1996.
% IGRF2000
% IAGA Working Group V-8 (Chair: Mioara Mandea, Institut de
% Physique du Globe de Paris, 4 Place Jussieu, 75252 Paris,
% France. Fax:33 238 339 504, email: mioara@ipgp.jussieu.fr).
% http://www.dsri.dk/Oersted/Field_models
%
% Maurice A. Tivey March 1997
% Mod Dec 1999 (add igrf2000 and y2k compliance
% Mod Nov 2000 (use up to degree 10 sh coefficients)
% Mod Apr 2005 added 2005 coeffs
% http://deeptow.whoi.edu/matlab.html
% Copyright: Maurice A. Tivey, 2005
% Woods Hole Oceanographic Institution
if nargin < 1
 disp('DEMO MAGFD:')
 help magfd
 disp('Compute magnetic field at Woods Hole from Jan 1st 1900');
 disp(' every five years until 2005');
 disp(' Latitude 42 N, Longitude 74W')
 disp('sample command: out=magfd(1997,1,0,90-42,-74);')
 for i=1:22,
 out(i,:)=magfd(-((i-1)*5+1900),1,0,90-42,-74); 
  end
 plot(([1:22]-1)*5+1900,out(:,4),'-r+','linewidth',2);
 xlabel('Year');
 ylabel('Total Magnetic Field (nT)')
 title('Total Magnetic Field Intensity at Woods Hole, MA')
 axis tight
 return
end
%igrfyear=2000;
%igrffile='sh2000';
%DGRF=[1900:5:2000];
DGRF=[1900:5:2015];
igrfyear=2005;
igrffile='sh2015';
pl=0;
if DATE < 0, pl=1; end
DATE=abs(DATE);
% Determine year for base DGRF to use.
 if DATE < igrfyear,
 BASE=fix(DATE-DGRF(1));
 i=fix(BASE/5)+1;
 BASE=DGRF(i);
 if pl==0,
 fprintf('Using DGRF base year %f \n',BASE);
 end
 eval(['load sh',num2str(BASE)])
 % loads agh and agh41 but now need to get
 iagh=agh;iagh41=agh41;
 % load next epoch
 eval(['load sh',num2str(DGRF(i+1))])
 eagh=agh;eagh41=agh41;
 dgh=(eagh-iagh)./5;dgh41=(eagh41-iagh41)./5;
 agh=iagh;agh41=iagh41;
 clear iagh iagh41 eagh eagh41
 T = DATE - BASE;
 else
% if pl==0,
% fprintf('Using IGRF base year %f \n',igrfyear);
% end
eval(['load ',igrffile]) % load in igrf data file
T = DATE - igrfyear;
 end
% combine spherical harmonic coefficients from first 8 degrees
% with degrees 9 and 10
 agh=[agh,agh41];
 dgh=[dgh,dgh41];
%
 D2R = pi/180;
 R = ALT;
 ONE = COLAT*0.01745329;
 SLAT = cos(ONE);
 CLAT = sin(ONE);
 ONE = ELONG*0.01745329;
 CL(1) = cos(ONE);
 SL(1) = sin(ONE);
 X = 0.0;
 Y = 0.0;
 Z = 0.0;
 CD = 1.0;
 SD = 0.0;
 L = 1;
 M = 1;
 N = 0; 
 if ITYPE == 1 % CONVERSION FROM GEODETIC TO GEOCENTRIC COORDINATES
 A2 = 40680925.; % squared semi major axis
 B2 = 40408588.; % squared semi minor axis
 ONE = A2*CLAT*CLAT;
 TWO = B2*SLAT*SLAT;
 THREE = ONE + TWO;
 FOUR = sqrt(THREE);
 R = sqrt(ALT*(ALT + 2.0*FOUR) + (A2*ONE + B2*TWO)/THREE);
 CD = (ALT + FOUR)/R;
 SD = (A2 - B2)/FOUR*SLAT*CLAT/R;
 ONE = SLAT;
 SLAT = SLAT*CD - CLAT*SD;
 CLAT = CLAT*CD + ONE*SD;
end
 RATIO = 6371.2/R;
%
% COMPUTATION OF SCHMIDT QUASI-NORMAL COEFFICIENTS P AND X(=Q)
%
 P(1) = 2.0*SLAT;
 P(2) = 2.0*CLAT;
 P(3) = 4.5*SLAT*SLAT - 1.5;
 P(4) = 5.1961524*CLAT*SLAT;
 Q(1) = -CLAT;
 Q(2) = SLAT;
 Q(3) = -3.0*CLAT*SLAT;
 Q(4) = 1.7320508*(SLAT*SLAT - CLAT*CLAT);
NMAX=10; % Max number of harmonic degrees
NPQ=(NMAX*(NMAX+3))/2;
for K=1:NPQ,
 if N < M
 M = 0;
 N = N + 1;
 RR = RATIO^(N + 2);
 FN = N;
 end
 FM = M;
 if K >= 5 %8,5,5
 if (M-N) == 0 %,7,6,7
 ONE = sqrt(1.0 - 0.5/FM);
 J = K - N - 1;
 P(K) = (1.0 + 1.0/FM)*ONE*CLAT*P(J);
 Q(K) = ONE*(CLAT*Q(J) + SLAT/FM*P(J));
 SL(M) = SL(M-1)*CL(1) + CL(M-1)*SL(1);
 CL(M) = CL(M-1)*CL(1) - SL(M-1)*SL(1);
 else
 ONE = sqrt(FN*FN - FM*FM);
 TWO = sqrt((FN - 1.0)^2 - FM*FM)/ONE;
 THREE = (2.0*FN - 1.0)/ONE;
 I = K - N;
 J = K - 2*N + 1;
 P(K) = (FN + 1.0)*(THREE*SLAT/FN*P(I) - TWO/(FN - 1.0)*P(J));
 Q(K) = THREE*(SLAT*Q(I) - CLAT/FN*P(I)) - TWO*Q(J);
 end
%
% SYNTHESIS OF X, Y AND Z IN GEOCENTRIC COORDINATES
%
 end
 ONE = (agh(L) + dgh(L)*T)*RR;
 if M == 0 %10,9,10
 X = X + ONE*Q(K);
 Z = Z - ONE*P(K);
 L = L + 1; 
  else
 TWO = (agh(L+1) + dgh(L+1)*T)*RR;
 THREE = ONE*CL(M) + TWO*SL(M);
 X = X + THREE*Q(K);
 Z = Z - THREE*P(K);
 if CLAT > 0 %12,12,11
 Y = Y+(ONE*SL(M)-TWO*CL(M))*FM*P(K)/((FN + 1.0)*CLAT);
 else
 Y = Y + (ONE*SL(M) - TWO*CL(M))*Q(K)*SLAT;
 end
 L = L + 2;
 end
 M = M + 1;
end
% CONVERSION TO COORDINATE SYSTEM SPECIFIED BY ITYPE
ONE = X;
X = X*CD + Z*SD;
Z = Z*CD - ONE*SD;
T = sqrt(X*X + Y*Y + Z*Z);
J=[X,Y,Z,T];
% END 