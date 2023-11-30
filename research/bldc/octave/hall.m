data = csvread("../data/hallChanges.csv");

hc = data(2:end, 1);

plot(hc);

dhc = diff(hc);

plot(dhc, "-*");

s = dhc(41:-1:1);

s1 = s - mean(s);
s2 = dhc - mean(dhc);

c = conv(s1, s2) / (42 * sqrt(var(s1)) * sqrt(var(s2)));
