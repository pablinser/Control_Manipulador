close all;
clear all;
%% Definición de la matriz dh y obtención del modelo cinemático directo
syms t1 t2 t3 t4 t5 t6 t7;
q1 = [t1 t2 t3 t4 t5 t6];
q2 = [t1 t2 t3 t4 t5 t6 t7];

% Matriz dh del manipulador
dh = [0 0 0 478 0; pi/2 50 0 226 0; 0 425 0 -176 0; -pi/2 0 0 425 0; pi/2 0 0 0 0; -pi/2 0 0 100 0];
% Matriz dh del manipulador incluyendo una falsa articulación
dh2 = [0 0 0 478 0; pi/2 50 0 226 0; -pi/2 0 0 425 0; pi/2 0 0 -176 0; -pi/2 0 0 425 0; pi/2 0 0 0 0; -pi/2 0 0 100 0];

% Obtención del modelo cinemático directo para las dos matrices dh
cd = simplify(fkine(dh, q1));
cd2 = simplify(fkine(dh2, q2));

%% trayectoria seguida mediante par computado
% La trayectoria seguida será la misma que la calculada en el apartado
% 'modelo cinemático inverso'. en este apartado se puede modificar el
% parámetro Tfin.
% Tfin es el tiempo en el que se desea que se recorra la circunferencia:
Tfin = 25; % En segundos.
% puntos contiene el número de puntos que se desea que tenga la trayectoria
puntos = 500;
% Generamos una trayectoria que haga el mismo recorrido pero cuyas
% velocidades y aceleraciones sean suaves.
t = 0: Tfin/(puntos-1): Tfin;

% radio de la circunferencia en mm
r = 500;
x = r*cos(-pi*(cos((pi/Tfin)*t)-1));
y = r*sin(-pi*(cos((pi/Tfin)*t)-1));
cd = ones(4,4,length(t));
for i = 1: length(t)
    cd(:,:,i) = [-1 0 0 x(i); 0 -1 0 y(i); 0 0 -1 0; 0 0 0 1];
end

Q = ikine(dh, 0.01, 10000, cd);
t = floor(Q./(2*pi));
Qts = 2*pi*ones(length(Q(:,1)), 6)*[min(t(:,1)) 0 0 0 0 0; 0 max(t(:,2)) 0 0 0 0; 0 0 max(t(:,3)) 0 0 0; 0 0 0 min(t(:,4)) 0 0; 0 0 0 0 max(t(:,5)) 0; 0 0 0 0 0 max(t(:,6))];
Ts = Tfin/(puntos-1);
t = 0: Tfin/(puntos-1): Tfin;
Qts = [t',Q-Qts];


figure
subplot (3,2,1)
plot (t,Q(:,1))
title ('theta1')
ylabel ('theta1 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,2)
plot (t,Q(:,2))
title ('theta2')
ylabel ('theta2 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,3)
plot (t,Q(:,3))
title ('theta3')
ylabel ('theta3 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,4)
plot (t,Q(:,4))
title ('theta4')
ylabel ('theta4 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,5)
plot (t,Q(:,5))
title ('theta5')
ylabel ('theta5 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,6)
plot (t,Q(:,6))
title ('theta6')
ylabel ('theta6 (rads)')
xlabel('vuelta (rads)')
figure
plot(x,y,'.');
title('trayectoria con velocidad suave');
figure
title('trayectoria con velocidad suave');
plotbot(dh, Q, 'w')

%% Seguimiento de una trayectoria real.
% Simularemos que queremos tomar un objeto dese la posición 1 [x,y,z] =
% [-200, -200, 0] y que deseamos soltarlo en la posición 2 [200, 200, 0].
% Para ello tendremos que definir una trayectoria desde el punto inicial
% en el que esté situado el manipulador hasta un punto de aproximación a la
% posición 1, que esté sobre esta. Después, se deberá recorrer una
% trayectoria hasta un punto de aproximación sobre la posición 2 y
% finalmente, soltar el objeto en la posición 2. Además, en los puntos de
% aproximación mantendremos un momento la referencia y tardará un segundo
% en alzanzar la velocidad máxima.

% Supondremos que el manipulador empieza con los siguientes valores en las
% variables articulares:

q = [0, pi/2, -pi/2, pi, 0, 0];


% definimos las posiciones p1, p2 y pa1, pa2. Estas últimas serán las
% posiciones de aproximación.
p1 = [-200 -200 0]';
p2 = [200 200 0]';
pa1 = [-200 -200 25]';
pa2 = [-200 -200 25]';

% Definimos el tiempo de reposo en cada posición de aproximación (en s)
Trep = 1;

% Definimos el tiempo de muestreo
Ts2 = 0.01;

% Definimos la velocidad máxima (mm/s):
Vm = 125;

% Obtenemos las variables articulares durante la trayectoria:
v = 0:Vm/((2/Ts2)-1): Vm;
p0 = fkine(dh, q);
p0 = p0(1:3,4);
dir = (pa1-p0)/sqrt(sum((pa1-p0).^2));
tray = ones(3,length(v))';
tray(1,:) = p0';

for i = 2:length(v)
    tray(i,:) = tray(i-1,:)+ dir'*v(i)*Ts2;
end

dir = (pa1-tray(end,:)');
tray = [tray; ones(floor(sqrt(sum(dir.^2))/(Vm*Ts2)), 3)];

for i = 41:floor(40+sqrt(sum(dir.^2))/(Vm*Ts2))
    tray(i, :) = tray (i-1,:)+(dir*Vm*Ts2/sqrt(sum(dir.^2)))';
end
    
tray = [tray; [ones(20,1)*pa1(1) ones(20,1)*pa1(2) ones(20,1)*pa1(3)]];

dir = (p1-tray(end,:)');
temp = length(tray);
tray = [tray; ones(floor(sqrt(sum(dir.^2))/((Vm/10)*Ts2)), 3)];
temp2 = length(tray);

for i = temp:temp2
    tray(i, :) = tray (i-1,:)+(dir*(Vm/10)*Ts2/sqrt(sum(dir.^2)))';
end
tray = [tray; [ones(20,1)*p1(1) ones(20,1)*p1(2) ones(20,1)*p1(3)]];
alpha = 0;
temp = length(tray);
i = 1;
while sum (alpha) < pi
    i = i+1;
    if i<length(v)
        alpha = alpha+(v(i)*Ts2)/200;
    else
        alpha = alpha+(Vm*Ts2)/200;
    end
    tray = [tray; [-200*cos(alpha) -200*cos(alpha) 200*sin(alpha)]];
end
tray = [tray; [ones(20,1)*p2(1) ones(20,1)*p2(2) ones(20,1)*p2(3)]];
tray2 = ones(4,4,length(tray));
for i = 1: length(tray)
    tray2(:,:,i) = [[-1 0 0; 0 -1 0; 0 0 -1; 0 0 0],[tray(i,:) 1]'];
end
%plotbot(dh, ikine(dh, 0.001, 10000, tray2), 'w');
Qt2 = ikine(dh, 0.001, 10000, tray2);
Qt2 = [[1:length(Qt2)]'*Ts2,Qt2];

Tfin2 = Qt2(end, 1);
%% Modelo cinemático inverso
% Emplearemos el modelo cinematico inverso del manipulador, mediante la
% función ikine, para que el efector final siga una trayectoria circular en
% el plano XY.


% puntos contiene el número de puntos que se desea que tenga la trayectoria
t = 0: 2*pi/(puntos-1): 2*pi;
% radio de la circunferencia en mm
r = 500;

x = r*cos(t);
y = r*sin(t);
cd = ones(4,4,length(t));

for i = 1 : length (t)
    cd(:,:,i) = [-1 0 0 x(i); 0 -1 0 y(i); 0 0 -1 0; 0 0 0 1];
end

Q = ikine(dh, 0.01, 10000, cd);
t = floor(Q./(2*pi));
Qt = 2*pi*ones(length(Q(:,1)), 6)*[min(t(:,1)) 0 0 0 0 0; 0 max(t(:,2)) 0 0 0 0; 0 0 max(t(:,3)) 0 0 0; 0 0 0 min(t(:,4)) 0 0; 0 0 0 0 max(t(:,5)) 0; 0 0 0 0 0 max(t(:,6))];
t = 0: Tfin/(puntos-1): Tfin;
Qt = [t',Q-Qt];


% Representamos las gráficas con los valores de las diferentes variables
% articulares a lo largo de la curva. En el eje y estará el valor tomado
% por la variable articular en cuestión y en el eje x estará el ángulo
% recorrido opor el efector final hasta el momento, en radianes ambas.
figure
plot (x, y)
title('Trayectoria seguida con z = 0')
figure
subplot (3,2,1)
plot (t,Q(:,1))
title ('theta1')
ylabel ('theta1 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,2)
plot (t,Q(:,2))
title ('theta2')
ylabel ('theta2 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,3)
plot (t,Q(:,3))
title ('theta3')
ylabel ('theta3 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,4)
plot (t,Q(:,4))
title ('theta4')
ylabel ('theta4 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,5)
plot (t,Q(:,5))
title ('theta5')
ylabel ('theta5 (rads)')
xlabel('vuelta (rads)')

subplot (3,2,6)
plot (t,Q(:,6))
title ('theta6')
ylabel ('theta6 (rads)')
xlabel('vuelta (rads)')

% Generamos una animación que muestre al manipulador siguiendo la
% trayectoria deseada.
% figure
% title('trayectoria circular');
% plotbot(dh, Q, 'w')

%% Obtención de la matriz de parámetros dinámicos (DYN)

% Definimos las masas de los diferentes enzaces. Además, supondremos que la
% fricción es nula en todas las articulaciones. Supondremos también que
% toda la masa de cada enlace está concentrada en su centro de masas, que
% hemos calculado de manera exacta en todas las piezas excepto en la pieza1
% (pz1 en Sketchup) debido a su complejidad geométrica, por lo que se ha
% optado por una aproximación.

m1 = 31.3;
m2 = 33.7;
m3 = 13.8;
m4 = 9.2;
m5 = 0.9;
% Asignamos una masa despreciable, pero no nula, al efector final.
m6 = 0.01;

dyn = [m1 -32 -47 -43 442177.7 477302.2 239638.7 0 0 0 0 7 0 0 0; m2 172 0 0 166761.2 1209436.8 1272312.2 0 0 0 0 7 0 0 0; m3 0 25 0 103670.2 63590.4 103670.2 0 0 0 0 7.5 0 0 0; m4 0 0 -131 69955 69955 25875 0 0 0 0 9.4 0 0 0; m5 0 16 0 2319.3 405 2319.3 0 0 0 0 8.3 0 0 0; m6 0 25 0 0 0 0 0 0 0 0 7.5 0 0 0];
dyn = [dh dyn];
err = randn(6,20).*sqrt(0.05);
dyne = (ones(6,20)+err).*dyn;

%% Representación gráfica del manipulador

q = [0, pi/2, -pi/2, pi, 0, 0];
figure
plotbot(dh, q, 'fw')

%% Comportamiento dinámico del manipulador
T0 = 0;
T1 = 1000;
g = [0 0 -9.81];
[T, Q, QD] = fdyn(dyn, T0, T1, g, 'torque1', [0, 0, 0, 0, 0, 0], [0 0 0 0 0 0]);

% Representamos el valor de las variables articulares a lo largo del tiempo
% integrado, podemos observar que el movimiento no decae con el tiempo.


figure
subplot (3,2,1)
plot (T,Q(:,1))
title ('theta1')
ylabel('theta1 (rads)')
xlabel('tiempo (s)')

subplot (3,2,2)
plot (T,Q(:,2))
title ('theta2')
ylabel ('theta2 (rads)')
xlabel('tiempo (s)')

subplot (3,2,3)
plot (T,Q(:,3))
title ('theta3')
ylabel ('theta3 (rads)')
xlabel('tiempo (s)')

subplot (3,2,4)
plot (T,Q(:,4))
title ('theta4')
ylabel ('theta4 (rads)')
xlabel('tiempo (s)')

subplot (3,2,5)
plot (T,Q(:,5))
title ('theta5')
ylabel ('theta5 (rads)')
xlabel('tiempo (s)')

subplot (3,2,6)
plot (T,Q(:,6))
title ('theta6')
ylabel ('theta6 (rads)')
xlabel('tiempo (s)')

% Generamos una animación para ver el comportamiento del manipulador frente
% a la gravedad en ausencia de par en todas las articulaciones. Como se han
% supuesto nulos los términos que definen la fricción en las
% articulaciones, el manipulador no llegará a pararse.

%figure
%title('comportamiento frente a gravedad');
%plotbot(dh, Q, 'fw')

% Simulamos el comportamiento del manipulador al aplicar 10 Nm en la
% segunda articulación.
T1 = 1000;
[T2, Q2, QD2] = fdyn(dyn, T0, T1, g, 'torque2', [0, -pi/2, -pi/2, pi, 0, 0], [0 0 0 0 0 0]);

% Representamos el valor de las variables articulares a lo largo del tiempo
% integrado, podemos observar que el movimiento no decae con el tiempo.

figure
subplot (3,2,1)
plot (T2,Q2(:,1))
title ('theta1')
ylabel ('theta1 (rads)')
xlabel('tiempo (s)')

subplot (3,2,2)
plot (T2,Q2(:,2))
title ('theta2')
ylabel ('theta2 (rads)')
xlabel('tiempo (s)')

subplot (3,2,3)
plot (T2,Q2(:,3))
title ('theta3')
ylabel ('theta3 (rads)')
xlabel('tiempo (s)')

subplot (3,2,4)
plot (T2,Q2(:,4))
title ('theta4')
ylabel ('theta4 (rads)')
xlabel('tiempo (s)')

subplot (3,2,5)
plot (T2,Q2(:,5))
title ('theta5')
ylabel ('theta5 (rads)')
xlabel('tiempo (s)')

subplot (3,2,6)
plot (T2,Q2(:,6))
title ('theta6')
ylabel ('theta6 (rads)')
xlabel('tiempo (s)')

%figure
%title('respuesta par constante');
%plotbot(dh, Q2, 'fw')