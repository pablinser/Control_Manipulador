% Cálculo del centro de masas de la pieza 3
% calculamos 'x', relacionada con la posición del centro de masas en el eje
% y.
r1 = 95;
r2 = 122;

h1 = 149;
h2 = 119;

d = 425;
B = asin((r2-r1)/d);
alpha = pi-2*B;


b = sqrt(d^2-(r2-r1)^2);

b1 = r1*sin(alpha/2);
b2 = r2*sin(alpha/2);
c1 = r1*cos(alpha/2);
c2 = r2*cos(alpha/2);

At1 = b1*c1;
At2 = b2*c2;

Acil1 = (r1^2)*pi;
Acil2 = (r2^2)*pi;

Vcil1 = Acil1*h1;
Vcil2 = Acil2*h1;

A1 = -At1 + 2*(r1+c1)*b1 + (r1+c1)*(r1+c1)/(d-c2+c1)-((2*pi-alpha)/(2*pi))*Acil1;
A2 = At2 + 2*(r2-c2)*(b1+(b2-b1)*(d+c1-r2)/(d-c2+c1)) + (r2-c2)*(r2-c2)/(d-c2+c1)-(alpha/(2*pi))*Acil2;
i = 0;
v1 = zeros (1001,1);
v2 = zeros (1001,1);
for x = 0:(d-r1-r2)/1000:d-r1-r2
    i = i+1;
    v1(i) = Vcil1 + A1*h2 + (2*(d-r1-r2-x)*b1 + (d+c1-r2-x)*(b2-b1)*((d+c1-r2-x)/(d-c2+c1))-(r1+c1)*(b2-b1)*((r1+c1)/(d-c2+c1)))*h2;
    v2(i) = Vcil2 + A2*h2 + (2*x*b1 + (b2-b1)*((d-c2+c1)-(r2-c2))/(d-c2+c1)-(b2-b1)*((d-c2+c1)-(r2-c2)-x)/(d-c2+c1))*h2;
end
figure
plot(0:(d-r1-r2)/1000:d-r1-r2, v1)
hold on
plot(0:(d-r1-r2)/1000:d-r1-r2, v2)
hold off
x = 0:(d-r1-r2)/1000:d-r1-r2;
disp('se cruzan en x =')
x(find(v2>v1,1))

% calculamos la posición del centro de masas de la pieza en el eje y. En el
% eje x no será necesario ya que es simétrica.
% y será la variable que emplearemos en este caso.
i = 0;
for y = 0:h2/1000:h2
    i = i+1;
    v1(i) = (Acil1+Acil2) * (h1-h2) + y * (Acil1+Acil2+54700);
    v2(i) = (h2-y) * (Acil1+Acil2+54700);
end

figure
plot(0:h2/1000:h2, v1)
hold on
plot(0:h2/1000:h2, v2)
hold off
y = 0:h2/1000:h2;
disp('se cruzan en y =')
y = y(find(v2>v1));
y = y(end)
