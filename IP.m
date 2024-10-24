% ----------- Solução para theta1 ----------- 
% Define a função para encontrar theta1 dada theta3
function F1 = theta1_equation(theta1, theta3)
    % Definindo a equação
    F1 = tan(theta3) * (324 - 114.3 * cos(theta1)) + 114.3 * sin(theta1);
end

% Valor de theta3 (em graus)
theta3_value = -10; % Por exemplo, 10 graus

% Convertendo theta3 para radianos
theta3_radians = deg2rad(theta3_value);

% Chute inicial para theta1 (em radianos)
theta1_initial_guess = 0.0; % Ajuste conforme necessário

% Usando fzero para encontrar theta1
theta1_solution = fzero(@(theta1) theta1_equation(theta1, theta3_radians), theta1_initial_guess);

% Convertendo theta1 para graus
theta1_degrees = rad2deg(theta1_solution);
fprintf('Solução para theta1: %.2f graus\n', theta1_degrees);

% ----------- Solução para theta2 ----------- 
% Define a função para encontrar theta2 dada theta4
function F2 = theta2_equation(theta2, theta4)
    % Usando a equação dada tan(theta4) = (2 * 57.15 * sin(theta2)) / 144
    F2 = tan(theta4) * 144 - 2 * 57.15 * sin(theta2);
end

% Valor de theta4 (em graus)
theta4_value = 10; % Exemplo: 10 graus

% Convertendo theta4 para radianos
theta4_radians = deg2rad(theta4_value);

% Chute inicial para theta2 (em radianos)
theta2_initial_guess = 0.0; % Ajuste conforme necessário

% Usando fzero para encontrar theta2
theta2_solution = fzero(@(theta2) theta2_equation(theta2, theta4_radians), theta2_initial_guess);

% Convertendo theta2 para graus
theta2_degrees = rad2deg(theta2_solution);
fprintf('Solução para theta2: %.2f graus\n', theta2_degrees);

