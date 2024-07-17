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
J_total = J + J_add; % = 4.24e-7 + 0.02 = 0.020000424 kg·m^2

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

% Calcular Kp y Kd basados en los parámetros deseados
% Parámetros deseados
zeta = 0.7;
omega_n = 2.86;

% Polos deseados
poles_dominant = [-zeta*omega_n + 1i*omega_n*sqrt(1 - zeta^2), -zeta*omega_n - 1i*omega_n*sqrt(1 - zeta^2)]

pole_non_dominant = -20;

% Polinomio deseado del sistema cerrado
desired_poles = [poles_dominant, pole_non_dominant];
desired_poly = poly(desired_poles)

% Coeficientes del polinomio deseado
a3 = desired_poly(1)
a2 = desired_poly(2)
a1 = desired_poly(3)
a0 = desired_poly(4)

% Ecuaciones para Kp y Kd con retroalimentación de 0.13
K_sensor = 0.13;


%b0 = K_sensor * Kt * Kp_value; % Coeficiente constante
%despejando para resolver Kp
Kp_value = a0/ (Kt * K_sensor);

%luego, quedan
% b1 = K_sensor * Kt * Kd_value + Kb * Kt + B * L + J * R; % Coeficiente de s^1
%despejando este ultimo:

Kd_value = (a1 - B*R - Kb*Kt) / (K_sensor * Kt) ;

fprintf('Kp: %f\n', Kp_value);
fprintf('Kd: %f\n', Kd_value);

% Definir el controlador PD
Kp = Kp_value;
Kd = Kd_value;
num_PD = [Kd, Kp];
den_PD = [1];

% Crear la función de transferencia del controlador PD
sys_PD = tf(num_PD, den_PD)

% Multiplicar el controlador PD por la función de transferencia del motor
sys_motor_PD = series(sys_PD, sys_int)

% Retroalimentación unitaria con ganancia 0.13
sys_h = tf([0.13], [0, 1]);
G_closed_PD = feedback(sys_motor_PD, sys_h)

% Simulación de la respuesta a un escalón de 12V
t = 0:0.01:1; % Tiempo de simulación de 0 a 10 segundos con paso de 0.01 segundos
[y_x, t] = step(12*G_closed_PD, t);

% Mostrar la función de transferencia y los polos del sistema cerrado
pole(G_closed_PD)
pzmap(G_closed_PD)
rlocus(G_closed_PD)

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

title('Respuesta en Posición Angular del Motor en los Ejes con Carga Adicional y Controlador PD');
xlabel('Tiempo (s)');
ylabel('Posición Angular (rad)');
grid on;
hold off;

% Calcular parámetros clave de la respuesta temporal
Mp = (max(y_x) - 92.295) / 92.295 * 100; % Sobreimpulso
tp = t(find(y_x == max(y_x), 1)); % Tiempo de pico

% Tiempo de estabilización (cuando la respuesta se mantiene dentro del 2% del valor final)
estabilidad = 0.02 * 92.295; % 2% del valor final

% Mostrar los resultados
fprintf('Tiempo de pico (tp): %.2f segundos\n', tp);
fprintf('Sobreimpulso (Mp): %.2f %%\n', Mp);


% Cálculo del error en estado estable
% La función de transferencia de lazo abierto tiene un polo en cero
% Error en estado estable para entrada escalón (función tipo 1) es 0

% Cálculo de Kp (Ganancia de posición)
Kp_sys = dcgain(sys_motor_PD);
error_steady_state = 12 / (1 + Kp_sys); % Error en estado estable

% Mostrar el error en estado estable calculado
fprintf('El error en estado estable calculado es: %.4f rad\n', error_steady_state);

% Mostrar Kp calculado
fprintf('La ganancia Kp calculada es: %.4f\n', Kp_sys);

