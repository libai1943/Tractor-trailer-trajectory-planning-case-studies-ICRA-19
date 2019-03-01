function s = area_cal(x, y)
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);

y1 = y(1);
y2 = y(2);
y3 = y(3);
y4 = y(4);

s = 0.5 * abs(x1 * y2 + x2 * y3 + x3 * y1 - x1 * y3 - x2 * y1 - x3 * y2) + 0.5 * abs(x1 * y4 + x4 * y3 + x3 * y1 - x1 * y3 - x4 * y1 - x3 * y4); 

