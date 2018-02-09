//Written by Rohan J. Dani
#include <vector>
#define std::vector;

double euclidianNorm(vector<int> v){
  double m_sum = 0.0;
  for (int i=1; i<=v.size();++i) {
      m_sum  += x[i]^2;
  }
  return sqrt(m_sum);
  }
void crossProduct(vector<int> A, vector<int> B, vector<int> P) {
  P[0] = A[1] * B[2] - A[2] * B[1];
  P[1] = A[0] * B[2] - A[2] * B[0];
  P[2] = A[0] * B[1] - A[1] * B[0];
}
void triad(vector<int> v1b,vector<int> v2b,vector<int> vli, vector<int> v2i) {
  int a;
  t1b=v1b;
  t2b=(crossProduct(v1b,v2b,t2b))/(euclidianNorm(crossProduct(v1i,v2i,t2b)));
  t3i=crossProduct(t1i,t2i,t3i);

  a=t1b.size();
    if a[1]==1 {
      //don't understand
      //Rbt=[t1b',t2b',t3b'];
      //Rit=[t1i',t2i',t3i'];
    }
    else {
      //Rbt=[t1b,t2b,t3b];
      //Rit=[t1i,t2i,t3i];
      //don't understand
    }
    Rbi = Rbt*Rit; //don't understand
}
