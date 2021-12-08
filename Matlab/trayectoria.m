
q0 = [0, pi/2, -pi/2, pi, 0, 0];
% Definimos el tiempo de muestreo (o tiempo de refresco en la referencia)
Ts2 = 0.01;
% definimos las posiciones p1, p2 y pa1, pa2. Estas últimas serán las
% posiciones de aproximación.
p1 = [700 0 0]';
p2 = [-700 0 0]';
pa1 = [700 0 50]';
pa2 = [-700 0 50]';

% Generamos la trayectoria desde la posición inicial a la posición pa1
t = 5; %tiempo que tarda en recorrer este segmento en segundos
puntos = floor(t/Ts2-1);
Q = ikine(dh, 1e-3, 10000, [[-1 0 0; 0 -1 0; 0 0 -1; 0 0 0],[pa1; 0]]);
trayq = [q0(1):(Q(1)-q0(1))/puntos:Q(1);q0(2):(Q(2)-q0(2))/puntos:Q(2);q0(3):(Q(3)-q0(3))/puntos:Q(3);q0(4):(Q(4)-q0(4))/puntos:Q(4);q0(5):(Q(5)-q0(5))/puntos:Q(5);q0(6):(Q(6)-q0(6))/puntos:Q(6)]';

%Generamos la trayectoria desde pa1 a p1
t = 2; %tiempo que tarda en recorrer este segmento en segundos
puntos = floor(t/Ts2-1);
trayx = ones(4,4,puntos+1); 
trayx (:,:,1) = [-1 0 0 700; 0 -1 0 0; 0 0 -1 50; 0 0 0 1];
i = 0;
for z = 50: -50/puntos: 0
    i = i+1;
    trayx (:,:,i) = [-1 0 0 700; 0 -1 0 0; 0 0 -1 z; 0 0 0 1];
end
temp = length(trayq);
trayq = [trayq; ikine(dh, 1e-3, 10000, trayx, trayq(end,:))];

% Trayectoria desde p1 a pa1

t = 1; %tiempo que tarda en recorrer este segmento en segundos
puntos = floor(t/Ts2-1);
trayx = ones(4,4,puntos+1); 
i = 0;
for z = 0: 50/puntos: 50
    i = i+1;
    trayx (:,:,i) = [-1 0 0 700; 0 -1 0 0; 0 0 -1 z; 0 0 0 1];
end
trayq = [trayq; ikine(dh, 1e-3, 10000, trayx, trayq(end,:))];

% tiempo en el que se recorrerá la circunferencia de pa1 a pa2
t = 10; 
puntos = floor(t/Ts2-1);
trayx = [trayq(end,1)+(pi/puntos)*[1:puntos]' trayq(end,2)*ones(puntos,1) trayq(end,3)*ones(puntos,1) trayq(end,4)*ones(puntos,1) trayq(end,5)*ones(puntos,1) trayq(end,6)*ones(puntos,1)];
trayq = [trayq; trayx];

% Trayectoria de aproximación de pa2 a p2

t = 2; %tiempo que tarda en recorrer este segmento en segundos
puntos = floor(t/Ts2-1);
trayx = ones(4,4,puntos+1);
t = fkine(dh,trayq(end,:));
i = 0;
for z = 50: -50/puntos: 0
    i = i+1;
    trayx (:,:,i) = [t(1,1) t(1,2) t(1,3) -700; t(2,1) t(2,2) t(2,3) 0; t(3,1) t(3,2) t(3,3) z; 0 0 0 1];
end
temp = length(trayq);
trayq = [trayq; ikine(dh, 1e-3, 10000, trayx, trayq(end,:))];


% Trayectoria para retirar el brazo de p2 hacia pa2
t = 2; %tiempo que tarda en recorrer este segmento en segundos
puntos = floor(t/Ts2-1);
trayx = ones(4,4,puntos+1); 
t = fkine(dh,trayq(end,:));
i = 0;
for z = 0: 50/puntos: 50
    i = i+1;
    trayx (:,:,i) = [t(1,1) t(1,2) t(1,3) -700; t(2,1) t(2,2) t(2,3) 0; t(3,1) t(3,2) t(3,3) z; 0 0 0 1];
end
trayq = [trayq; ikine(dh, 1e-3, 10000, trayx, trayq(end,:))];
Qt2 = [[0:length(trayq)-1]'*Ts2 trayq];
Tfin2 = length(trayq)*Ts2;

%plotbot(dh, trayq, 'wr')