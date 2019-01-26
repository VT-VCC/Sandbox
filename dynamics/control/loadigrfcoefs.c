//Written by Rohan J. Dani
//loads the coefficients in the igrf model
//and returns matrices g and h from igrfcoefs file
#include <time.h>
#include <string.h>
#include <string>
#include <stdio.h>
#include <errno.h>
#include <vector>

void createMatrix(std::vector<int> x) {
  for(i = 0; i < nrows; ++i)
{
   for(j = 0; j < ncols ; ++j)
   {
      printf("%d\t",x[i][j]);
   }
printf("\n");
}
}

int loadigrfcoefs(std::string timeString) {
  int lastepoch;
  int nextepoch;
  std::vector<int> g;
  std::vector<int> h;
  std::vector<string> timeVector;
  FILE *fp;

  if(isalpha(timeString)) {
    time_t rawTime;
    struct tm *timeString;
    char buffer[80];

    time(&rawTime);
    timeString = localtime(&rawTime);
    timeString = strftime(buffer,80,"%x",timeString); //convert to standard time format
    }
  if(strlen(timeString) > 1) {
    perror("Error: Time can only have one element");
  }
  for(int i = 0; i < strlen(timeString); ++i) {
    timeVector[i] = timeString[i];
  }
  //time = timevec(1) + (time - datenum([timevec(1) 1 1]))./(365 + double(...
    //(~mod(timevec(1),4) & mod(timevec(1),100)) | (~mod(timevec(1),400))));
  //timeString = timeVector[1] + (timeString - )  Don't know how to convert this to C

  fp = fopen("igrfcoefs.c");

  //check time validity

  if(time < years[1] || time > years[strlen[years]]) {
    perror("igrf:timeOutofRange, it is only valid between beginning and end of years");
  }

  for(int j = 0; j < (years - time)); ++j) {
    if ((years-time) == 1){
      lastepoch++;
    }
  }
  nextepoch = lastepoch + 1;

  g = createMatrix(g);
  h = createMatrix(h);


}
