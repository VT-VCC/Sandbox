function [JD]=Date2Julian(year,month,day,hour,minute,second)
%Takes input of current date and converts to Julian Date
Term1=367*year;
Term2=floor(7*(year+floor((month+9)/12))/4);
Term3=floor(275*month/9);

JD=Term1-Term2+Term3+day+1721013.5+hour/24+minute/1440+second/86400;
end