#include "reduceRadians.c"
int Date2Julian(int year,int month,int day,int hour, int minute, int second) {
  //take input of current date and convert to Julian date
  int finalVal = 0;
  int Term1 = 367*year;
  int Term2=floor(7*(year+floor((month+9)/12))/4);
  int Term3=floor(275*month/9);

  finalVal = Term1-Term2+Term3+day+1721013.5+hour/24+minute/1440+second/86400;
  return finalVal;
}
