%% trabajoRob3.m
% Este fichero se debe ejecutar tras la ejecución de TrayectoriaReal.slx, ya
% que toma los resultados de éste y los visualiza.

% En primer lugar, visualizaremos los errores.
close all
figure
subplot (3,2,1)
plot (time2(2:end),qd2(1:end-1,1)-q2(2:end,1))
title ('theta1')
ylabel ('Error 1 (rads)')
xlabel('t (s)')

subplot (3,2,2)
plot (time2(2:end),qd2(1:end-1,2)-q2(2:end,2))
title ('theta2')
ylabel ('Error 2 (rads)')
xlabel('t (s)')


subplot (3,2,3)
plot (time2(2:end),qd2(1:end-1,3)-q2(2:end,3))
title ('theta3')
ylabel ('Error 3 (rads)')
xlabel('t (s)')


subplot (3,2,4)
plot (time2(2:end),qd2(1:end-1,4)-q2(2:end,4))
title ('theta4')
ylabel ('Error 4 (rads)')
xlabel('t (s)')


subplot (3,2,5)
plot (time2(2:end),qd2(1:end-1,5)-q2(2:end,5))
title ('theta5')
ylabel ('Error 5 (rads)')
xlabel('t (s)')


subplot (3,2,6)
plot (time2(2:end),qd2(1:end-1,6)-q2(2:end,6))
title ('theta6')
ylabel ('Error 6 (rads)')
xlabel('t (s)')

% Ahora representamos las referencias junto con la trayectoria seguida
% (línea discontinua)
figure

subplot (3,2,1)
plot (time2,qd2(:,1))
hold on
plot (time2,q2(:,1), 'r:')
title ('theta1')
ylabel ('theta1 (rads)')
xlabel('t (s)')

subplot (3,2,2)
plot (time2,qd2(:,2))
hold on
plot (time2,q2(:,2), 'r:')
title ('theta2')
ylabel ('theta2 (rads)')
xlabel('t (s)')


subplot (3,2,3)
plot (time2,qd2(:,3))
hold on
plot (time2,q2(:,3), 'r:')
title ('theta3')
ylabel ('theta3 (rads)')
xlabel('t (s)')


subplot (3,2,4)
plot (time2,qd2(:,4))
hold on
plot (time2,q2(:,4), 'r:')
title ('theta4')
ylabel ('theta4 (rads)')
xlabel('t (s)')


subplot (3,2,5)
plot (time2,qd2(:,5))
hold on
plot (time2,q2(:,5), 'r:')
title ('theta5')
ylabel ('theta5 (rads)')
xlabel('t (s)')


subplot (3,2,6)
plot (time2,qd2(:,6))
hold on
plot (time2,q2(:,6), 'r:')
title ('theta6')
ylabel ('theta6 (rads)')
xlabel('t (s)')

% representamos la trayectoria seguida realmente (discontinua) y la
% referencia (continua)

Td = fkine(dh, qd2);
Tr = fkine(dh, q2);
td = ones(length(Qt2(:,1)), 3);
tr = ones(length(Qt2(:,1)), 3);

for i = 1: length(qd2)
    td(i, 1) = Td(1,4,i);
    td(i, 2) = Td(2,4,i);
    td(i, 3) = Td(3,4,i);
    tr(i, 1) = Tr(1,4,i);
    tr(i, 2) = Tr(2,4,i);
    tr(i, 3) = Tr(3,4,i);
end

figure
plot3 (td(:,1), td(:,2), td(:,3))
hold on
grid on
plot3 (tr(:,1), tr(:,2), tr(:,3), 'r:')
xlabel ('x')
ylabel ('y')
zlabel ('z')
title('referencia vs trayectoria seguida')
disp('error máximo en valor absoluto (mm):')
max(sqrt(sum((td(1:end-1,:)-tr(2:end,:)).^2,2)))
disp('media del error en valor absoluto')
mean(sqrt(sum((td(1:end-1,:)-tr(2:end,:)).^2,2)))
figure
plot(time2(2:end,:), sqrt(sum((td(1:end-1,:)-tr(2:end,:)).^2,2)))
title('error en valor absoluto');
xlabel ('tiempo (s)');
ylabel ('error (mm)')

figure
plot(td(:,1),td(:,2),'.')
hold on
plot(tr(:,1),tr(:,2),'r.')
xlabel('x')
ylabel('y')
title('proyección de la trayectoria en el plano XY')
figure
plotbot(dh, q2, 'w');