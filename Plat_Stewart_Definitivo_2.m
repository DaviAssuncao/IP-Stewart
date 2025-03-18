clc; clear; close all;

% Definição de parâmetros do sistema
M = 0.3;   % Massa da bolinha (kg)
g = 9.81;   % Gravidade (m/s²)
dt = 0.1;  % Passo de tempo (s)
Tmax = 15;  % Tempo máximo da simulação

% Ganhos PID
Kp = 5;   % Proporcional
Ki = 3;   % Integral
Kd = 1;   % Derivativo

% Posição inicial da bolinha
x = 0.1;  
y = -0.15;
vx = 0; vy = 0; % Velocidade inicial
ax = 0; ay = 0; % Aceleração inicial

% Posição final desejada (pode ser alterada)
x_ref = input('Digite a posição desejada em X: ');
y_ref = input('Digite a posição desejada em Y: ');

% Definição de erro inicial
erro_x_anterior = x_ref - x;
erro_y_anterior = y_ref - y;
integral_x = 0; 
integral_y = 0;

% Vetores para armazenar valores
T = 0:dt:Tmax;
pos_x = zeros(size(T));
pos_y = zeros(size(T));

% Configuração da animação
figure(1);
axis([-0.5 0.5 -0.5 0.5]);
grid on; hold on;
ball = plot(x, y, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('X (m)'); ylabel('Y (m)');
title('Animação da bolinha convergindo para o ponto desejado');

% Cabeçalho para exibição no console
fprintf('Tempo (s)   X (m)    Y (m)    Theta_x (°)    Theta_y (°)\n');
fprintf('------------------------------------------------------\n');

for i = 1:length(T)
    % Cálculo do erro
    erro_x = x_ref - x;
    erro_y = y_ref - y;

    % Ajuste de sinal conforme quadrante
    if x < x_ref && y > y_ref  % Quadrante 2
        erro_x = abs(erro_x);
        erro_y = -abs(erro_y);
    elseif x < x_ref && y < y_ref  % Quadrante 3
        erro_x = abs(erro_x);
        erro_y = abs(erro_y);
    elseif x > x_ref && y < y_ref  % Quadrante 4
        erro_x = -abs(erro_x);
        erro_y = abs(erro_y);
    end

    % Controle PID gerando FORÇA
    integral_x = integral_x + erro_x * dt;
    integral_y = integral_y + erro_y * dt;

    derivativo_x = (erro_x - erro_x_anterior) / dt;
    derivativo_y = (erro_y - erro_y_anterior) / dt;

    theta_x =(Kp * erro_x + Ki * integral_x + Kd * derivativo_x);
    theta_y =(Kp * erro_y + Ki * integral_y + Kd * derivativo_y);

    % Conversão para graus
    theta_x_deg = rad2deg(theta_x);
    theta_y_deg = rad2deg(theta_y);

    % Cálculo da aceleração
    ax = (-g * sin(theta_x));
    ay = (-g * sin(theta_y));

    % Integração usando Runge-Kutta (ode45)
    [~, V] = ode45(@(t, v) [ax; ay], [0 dt], [vx; vy]);
    vx = V(end, 1);
    vy = V(end, 2);

    [~, P] = ode45(@(t, p) [vx; vy], [0 dt], [x; y]);
    x = P(end, 1);
    y = P(end, 2);

    % Armazena os valores
    pos_x(i) = x;
    pos_y(i) = y;

    % Atualiza erro anterior
    erro_x_anterior = erro_x;
    erro_y_anterior = erro_y;

    % Exibir posição e ângulos no console
    fprintf('%.2f        %.5f   %.5f   %.2f°   %.2f°\n', T(i), x, y, theta_x_deg, theta_y_deg);

    % Atualiza a animação
    set(ball, 'XData', x, 'YData', y);
    pause(0.01);
end
hold off;

% Gráfico da trajetória da bolinha
figure(2);
plot(pos_x, pos_y, 'b-', 'LineWidth', 1.5);
hold on;
plot(x_ref, y_ref, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Posição X'); ylabel('Posição Y');
title('Trajetória da bolinha no espaço (x, y)');
grid on;

% Gráfico da convergência de X e Y ao longo do tempo
figure(3);
plot(T, pos_x, 'r', 'LineWidth', 1.5); hold on;
plot(T, pos_y, 'b', 'LineWidth', 1.5);
legend('Posição X', 'Posição Y');
xlabel('Tempo (s)');
ylabel('Posição');
title('Convergência de X e Y ao longo do tempo');
grid on;
