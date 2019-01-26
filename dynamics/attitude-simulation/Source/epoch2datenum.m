function [DateNumber] = epoch2datenum(eD, eY)

dayofyear = floor(eD);
decimal_time = rem(eD,1);

leap_year = false;

year = eY + 2000;
rem4 = mod(year, 4);

if (rem4 == 0)
    leap_year = true;
end

if leap_year
    month_start = [31 29 31 30 31 30 31 31 30 31 30 31];
else
    month_start = [31 28 31 30 31 30 31 31 30 31 30 31];
end

n = length(month_start);
sum = 1;
found = false;

for i = 1:n
    sum = sum + month_start(i);
    if (sum > dayofyear) && (~found)
        month = i;
        dayofmonth = dayofyear - sum + month_start(i) + 1;
        found = true;
    end
end

hour = floor(decimal_time*24);
decimal_time = (decimal_time*24) - hour;

min = floor(decimal_time*60);
decimal_time = (decimal_time*60) - min;

sec = decimal_time*60;

DateNumber = datenum(year, month, dayofmonth, hour, min, sec);

end