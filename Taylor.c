struct taylorStruct {
  int ySin;
  int yCos;
  int varianceSin;
  int varianceCos;
}

int Taylor(int x,int y) {
  taylorStruct taylor;
  taylor.ySin = 0;
  taylor.yCos = 0;
  int errorSin = 0;
  int errorCos = 0;
  for(int i = 0; i < k; i++){
    ySin=ySin+((-1)^(n-1)*(x.^(2*n-1))/(factorial(2*n-1)));
    yCos=yCos+((-1)^(n-1)*(x.^(2*(n-1)))/(factorial(2*(n-1))));
  }
  errorSin = ySin-sin(x);
  taylor.varianceSin = sqrt(errorSin*errorSin);
  errorCos = yCos-cos(x);
  taylor.varianceCos = sqrt(errorCos*errorCos);
  return taylor;
}
