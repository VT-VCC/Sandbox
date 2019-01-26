function [BbmBit,Bbm] = magErr(Bb,CalData,sig)

%Simulate magneometer reading with noise.  fmt in bit value 
fmtA = round((Bb(:,1)-CalData(1,2))/CalData(1,1)+sig*randn(3,1));  %Front  Magnetometer Axis A (+X) 
fmtB = round((Bb(:,3)-CalData(2,2))/CalData(2,1)+sig*randn(3,1));  %Front  Magnetometer Axis B (+Z) 
lmtA = round((-Bb(:,2)-CalData(3,2))/CalData(3,1)+sig*randn(3,1)); %Left   Magnetometer Axis A (-Y) 
lmtB = round((-Bb(:,3)-CalData(4,2))/CalData(4,1)+sig*randn(3,1)); %Left   Magnetometer Axis B (-Z) 
rmtA = round((-Bb(:,2)-CalData(5,2))/CalData(5,1)+sig*randn(3,1)); %Right  Magnetometer Axis A (-Y) 
rmtB = round((Bb(:,3)-CalData(6,2))/CalData(6,1)+sig*randn(3,1));  %Right  Magnetometer Axis B (+Z) 
tmtA = round((-Bb(:,2)-CalData(7,2))/CalData(7,1)+sig*randn(3,1)); %Top    Magnetometer Axis A (-Y) 
tmtB = round((-Bb(:,1)-CalData(8,2))/CalData(8,1)+sig*randn(3,1)); %Top    Magnetometer Axis B (-X) 
bmtA = round((-Bb(:,2)-CalData(9,2))/CalData(9,1)+sig*randn(3,1)); %Bottom Magnetometer Axis A (-Y) 
bmtB = round((Bb(:,1)-CalData(10,2))/CalData(10,1)+sig*randn(3,1));%Bottom Magnetometer Axis B (+X)  

xmt = median([fmtA';255-tmtB';bmtB']);                    %Average of X-Axis magnetometer Readings 
ymt = median([255-lmtA';255-rmtA';255-tmtA';255-bmtA']);  %Average of Y-Axis magnetometer Readings 
zmt = median([fmtB';255-lmtB';rmtB']);                    %Average of Z-Axis magnetometer Readings  

BbmBit = [xmt',ymt',zmt'];                   %Measured Magnetic Field as implemented on CP2 (bits)  

%Simulate conversion by satellite into Tesla 
mt = [fmtA fmtB lmtA lmtB rmtA rmtB tmtA tmtB bmtA bmtB]'; 
mtCal = [CalData(:,1) CalData(:,1) CalData(:,1)].*mt + [CalData(:,2) CalData(:,2) CalData(:,2)];  

fmtAcal = mtCal(1,:); fmtBcal = mtCal(2,:); lmtAcal = mtCal(3,:); lmtBcal = mtCal(4,:); 
rmtAcal = mtCal(5,:); rmtBcal = mtCal(6,:); tmtAcal = mtCal(7,:); 
tmtBcal = mtCal(8,:); bmtAcal = mtCal(9,:); bmtBcal = mtCal(10,:);  

xmtCal = median([fmtAcal;-tmtBcal;bmtBcal]);            %Average of X-Axis magnetometer Readings 
ymtCal = median([-lmtAcal;-rmtAcal;-tmtAcal;-bmtAcal]); %Average of Y-Axis magnetometer Readings 
zmtCal = median([fmtBcal;-lmtBcal;rmtBcal]);            %Average of Z-Axis magnetometer Readings  

Bbm = [xmtCal',ymtCal',zmtCal'];          %Measured Magnetic Field in the Body Frame (Tesla)  