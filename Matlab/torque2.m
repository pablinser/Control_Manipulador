% Función que aplica un par nulo a todas las articulaciones para ver el
% comportamiento del manipulador frente a la gravedad.
function tau = torque2 (t,x) 
tau=[0; 100000; 0; 0; 0; 0];