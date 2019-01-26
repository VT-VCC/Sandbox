function [ySin,yCos,varianceSin,varianceCos]=Taylor(x,k)
ySin=0;
yCos=0;
  for n=1:k
    ySin=ySin+((-1)^(n-1)*(x.^(2*n-1))/(factorial(2*n-1)));
    yCos=yCos+((-1)^(n-1)*(x.^(2*(n-1)))/(factorial(2*(n-1))));
  end
%   figure
 errorSin=ySin-sin(x);
 varianceSin=sqrt(errorSin*errorSin');
  errorCos=yCos-cos(x);
 varianceCos=sqrt(errorCos*errorCos');
% plot(x,ySin,'k')
% hold on
% plot(x,yCos,'r')
% plot(x,cos(x),'g')
% plot(x,sin(x),'b')