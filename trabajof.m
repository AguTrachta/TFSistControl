close all; clear all; clc;
pkg load control;
% Parámetros del sistema
L = 2.32e-3; % Inductancia del motor en H
R = 3.8; % Resistencia del motor en Ω
Kt = 0.1156; % Constante de torque en Nm/A
J = 4.24e-7; % Momento de inercia del rotor en kg·m^2
B = 0.000276; % Coeficiente de fricción viscosa en Nm·s/rad
Kb = 0.1156; % Constante de fuerza contraelectromotriz en V·s/rad

% Parámetros adicionales para el eje X
m = 0.5; % Masa en kg
d = 0.20; % Distancia en metros

% Momento de inercia adicional
J_add = m * d^2; % = 0.5 * (0.20^2) = 0.02 kg·m^2

% Momento de inercia total
J_total = J + J_add % = 4.24e-7 + 0.02 = 0.020000424 kg·m^2

% Función de transferencia del motor con carga adicional
num_motor = Kt;
den_motor = [L*J_total, L*B + J_total*R, B*R + Kt*Kb];

% Crear la función de transferencia completa del motor
sys_motor = tf(num_motor, den_motor);

% Retroalimentación unitaria multiplicada por Kb (lazo cerrado sobre posición angular)
sys_closed = feedback(sys_motor, Kb);

% Añadir el integrador para obtener la posición angular
sys_integrator = tf([1], [1, 0]);
sys_int = series(sys_closed, sys_integrator)

%bode(sys_int)
%rlocus(sys_int)
sys_h = tf([0.13], [0, 1])
% Retroalimentación unitaria
G_closed = feedback(sys_int, 0.13);

% Simulación de la respuesta a un escalón de 1V
t = 0:0.01:70; % Tiempo de simulación de 0 a 70 segundos con paso de 0.01 segundos
[y_x, t] = step(12*G_closed, t);

G_closed
pole(G_closed)
%pzmap(G_closed)
%rlocus(G_closed)

% Gráfica de la respuesta en posición angular
figure;
plot(t, y_x);
hold on;
% Línea horizontal en y = 92.295
line([0, max(t)], [92.295, 92.295], 'Color', 'r', 'LineStyle', '--');
text(0.5, 92.295, '92.295', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'r');

% Línea horizontal en y = 90.44
line([0, max(t)], [90.44, 90.44], 'Color', 'g', 'LineStyle', '--');
text(0.5, 90.44, '90.44', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'g');

% Línea horizontal en y = 94.1
line([0, max(t)], [94.1, 94.1], 'Color', 'b', 'LineStyle', '--');
text(0.5, 94.1, '94.1', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'left', 'Color', 'b');
title('Respuesta en Posición Angular del Motor en los Ejes con Carga Adicional');
xlabel('Tiempo (s)');
ylabel('Posición Angular (rad)');
grid on;
hold off;

% Calcular parámetros clave de la respuesta temporal
M = max(y_x)
Mp = (max(y_x) - 92.295) / 92.295 * 100; % Sobreimpulso
tp = t(find(y_x == max(y_x), 1)); % Tiempo de pico

% Tiempo de estabilización (cuando la respuesta se mantiene dentro del 2% del valor final)
estabilidad = 0.05 * 92.295; % 2% del valor final
%t_s = t(find(abs(y_x - 92.295) < estabilidad, 1, 'last'));

% Mostrar los resultados
fprintf('Tiempo de pico (tp): %.2f segundos\n', tp);
fprintf('Sobreimpulso (Mp): %.2f %%\n', Mp);
%fprintf('Tiempo de estabilización (ts): %.2f segundos\n', t_s);

% Cálculo del factor de amortiguamiento (zeta) y la frecuencia natural (omega_n)
zeta = sqrt( 1 / (1 + (pi / -log(Mp/100))^2) );
omega_n = pi / (tp * sqrt(1 - zeta^2));

fprintf('Factor de amortiguamiento (zeta): %.3f\n', zeta);
fprintf('Frecuencia natural (omega_n): %.3f rad/s\n', omega_n);

% Cálculo del error en estado estable
% La función de transferencia de lazo abierto tiene un polo en cero
% Error en estado estable para entrada escalón (función tipo 1) es 0

% Cálculo de Kp (Ganancia de posición)
Kp = dcgain(sys_int);
error_steady_state = 12 / (1 + Kp); % Error en estado estable

% Gráfica del error en estado estable
figure;
error = (0.1156/0.01503)*12 - y_x; % Error en cada punto de tiempo
plot(t, error);
title('Error en Estado Estable del Motor en los Ejes con Carga Adicional');
xlabel('Tiempo (s)');
ylabel('Error (rad)');
grid on;

% Mostrar el error en estado estable calculado
fprintf('El error en estado estable calculado es: %.4f rad\n', error_steady_state);

% Mostrar Kp calculado
fprintf('La ganancia Kp calculada es: %.4f\n', Kp);

