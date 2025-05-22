% Definição de variáveis simbólicas para massa e momentos de inércia
syms m Ixx Iyy Izz

% Função que implementa o método de Runge-Kutta de 4ª ordem
% para resolver equações diferenciais de sistemas dinâmicos.
% Parâmetros:
%   odefun - Função da EDO (Equação Diferencial Ordinária)
%   u - Entradas do sistema ao longo do tempo
%   x0 - Condição inicial do vetor de estados
%   t - Vetor de tempo para a integração
function Xpt = rungekutta(odefun,u,x0,t)
    % Número de passos de tempo
    nSteps = length(t);
    
    % Número de estados do sistema
    nStates = length(x0);
    
    % Inicializa a matriz de estados com zeros
    Xpt = zeros(nSteps, nStates);
    
    % Atribui a condição inicial ao primeiro passo
    Xpt(1, :) = x0;
    
    % Passo de tempo assumindo espaçamento uniforme no vetor t
    dt = t(2) - t(1);
    
    % Iteração sobre os passos de tempo
    for i = 1:nSteps-1
        % Estado atual
        x = Xpt(i, :)';
        
        % Entrada atual do sistema
        ui = u(:, i);
        
        % Cálculo da derivada utilizando a função da EDO
        dx = odefun(t(i), x, ui);
        
        % Coeficientes do método de Runge-Kutta de 4ª ordem
        k1 = dx;
        k2 = odefun(t(i) + dt/2, x + k1 * dt/2, ui);
        k3 = odefun(t(i) + dt/2, x + k2 * dt/2, ui);
        k4 = odefun(t(i) + dt, x + k3 * dt, ui);
        
        % Atualiza o estado utilizando a fórmula de Runge-Kutta 4
        Xpt(i+1, :) = (x + dt/6 * (k1 + 2*k2 + 2*k3 + k4))';
    end
end

% Definição da massa e momentos de inércia
m = 1;
Ixx = 1; Iyy = 1; Izz = 1;

% Definição da função da EDO (dinâmica do sistema)
odefun = @(t, x, u) [...
    x(9)*sin(x(5)) + x(7)*cos(x(5))*cos(x(6)) - x(8)*cos(x(5))*sin(x(6));
    x(7)*(cos(x(4))*sin(x(6)) + cos(x(6))*sin(x(4))*sin(x(5))) + ...
    x(8)*(cos(x(4))*cos(x(6)) - sin(x(4))*sin(x(5))*sin(x(6))) - ...
    x(9)*cos(x(5))*sin(x(4));
    x(7)*(sin(x(4))*sin(x(6)) - cos(x(4))*cos(x(6))*sin(x(5))) + ...
    x(8)*(cos(x(6))*sin(x(4)) + cos(x(4))*sin(x(5))*sin(x(6))) + ...
    x(9)*cos(x(4))*cos(x(5));
    x(11)*cos(x(6)) + x(10)*sin(x(6));
    -(x(11)*sin(x(6))^2 - x(10)*cos(x(6))*sin(x(6))) / (cos(x(5))*sin(x(6)));
    (x(11)*sin(x(5))*sin(x(6))^2 + x(12)*cos(x(5))*sin(x(6)) - ...
    x(10)*cos(x(6))*sin(x(5))*sin(x(6))) / (cos(x(5))*sin(x(6)));
    x(8)*x(12) - x(9)*x(11) + (u(3,:) * sin(x(5)) + u(1,:) * cos(x(5)) * cos(x(6)) - u(2,:) * cos(x(5)) * sin(x(6))) / m;
    x(9)*x(10) - x(7)*x(12) + (u(1,:) * (cos(x(4))*sin(x(6)) + cos(x(6))*sin(x(4))*sin(x(5))) + ...
    u(2,:) * (cos(x(4))*cos(x(6)) - sin(x(4))*sin(x(5))*sin(x(6))) - u(3,:) * cos(x(5)) * sin(x(4))) / m;
    x(7)*x(11) - x(8)*x(10) + (u(1,:) * (sin(x(4))*sin(x(6)) - cos(x(4))*cos(x(6))*sin(x(5))) + ...
    u(2,:) * (cos(x(6))*sin(x(4)) + cos(x(4))*sin(x(5))*sin(x(6))) + u(3,:) * cos(x(4)) * cos(x(5))) / m;
    ... % Continua com as equações das velocidades angulares
];

% Definição do intervalo de tempo para a simulação
tspan = 0:0.09:15;

% Estado inicial do sistema
x0 = ones(12,1); % Vetor inicial de estados preenchido com 1

% Entradas de controle ao longo do tempo
u = zeros(6, length(tspan)); % Inicializa com zeros
u(1, :) = 3; % Define um valor fixo para a primeira entrada

% Resolver a EDO utilizando o método de Runge-Kutta
Xpt = rungekutta(odefun, u, x0, tspan);

% Gerar gráficos dos estados ao longo do tempo
numStates = size(Xpt, 2);
for i = 1:numStates
    figure;
    plot(tspan, Xpt(:, i), 'LineWidth', 1.5);
    xlabel('Tempo (s)', 'FontWeight', 'bold');
    ylabel(['Estado ' num2str(i)], 'FontWeight', 'bold');
    title(['Gráfico do Estado ' num2str(i)], 'FontWeight', 'bold', 'FontSize', 12);
    grid on;
    xlim([min(tspan) max(tspan)]);
end
