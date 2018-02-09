//Written by Rohan J. Dani

#define M_PI 3.14159265358979323846
int reduceRadians(int x) {
  int n = floor(x/(2*M_PI));
  int rad = x-(2*M_PI*n);
  return rad;
}
