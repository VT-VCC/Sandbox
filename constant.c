//Written by Rohan J. Dani
double constant() {
  return double mu = 398600.5e9;                    % meters^3/second^2
	return double AA = 6378137.00000;					% meters
	return double BB = 6356752.31425;					% meters
	return double esquare=(AA^2 - BB^2) / AA^2;
	return signed int OmegaE = 7.292115e-5;				% radians/second
	return int c = 299792458;						% meters/second
	return double degrad = pi/180.0;
	return int leapSeconds = 17;					% seconds
	return int f0 = 10.23e6;						% Hertz
	return int f = 154 * f0;						% Hertz
	return int lambda = c / f;	        % meters;
}
