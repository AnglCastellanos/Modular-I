function compararTiroParabolico()
    % Parámetros comunes
    g = 9.81; % aceleración debido a la gravedad (m/s^2)
    m = 0.057; % masa de la pelota de tenis (kg)
    r = 0.0335; % radio de la pelota de tenis (m)
    A = pi * r^2; % área de referencia (m^2)
    tiempo_final = 10; % tiempo de simulación final (s)
    dt = 0.01; % paso de tiempo (s)
    theta = deg2rad(45); % ángulo de lanzamiento
    v0 = 50; % velocidad inicial (m/s)
    
    % Simulación sin arrastre
    [x_sin, y_sin, distancia_sin, tiempo_vuelo_sin] = simulacionSinArrastre(theta, v0, m, g, tiempo_final, dt);
    
    % Simulación con arrastre
    [x_con, y_con, distancia_con, tiempo_vuelo_con] = simulacionConArrastre(theta, v0, m, r, A, g, tiempo_final, dt);
    
    % Gráfica de ambas simulaciones
    figure;
    plot(x_sin, y_sin, 'LineWidth', 2, 'Color', [0, 0.5, 1]);  % Sin arrastre
    hold on;
    plot(x_con, y_con, 'LineWidth', 2, 'Color', [1, 0.5, 0]);  % Con arrastre
    title('Comparación del Tiro Parabólico con y sin Resistencia del Aire');
    xlabel('Distancia (m)');
    ylabel('Altura (m)');
    legend('Sin Resistencia del Aire', 'Con Resistencia del Aire');
    grid on;
    
    % Información adicional
    texto_info_sin = sprintf(['Sin Arrastre\nVelocidad Inicial: %.2f m/s\nÁngulo: %.2f grados\n' ...
        'Altura Máxima: %.2f m\nDistancia Máxima: %.2f m\nTiempo de Vuelo: %.2f s'], ...
        v0, rad2deg(theta), max(y_sin), distancia_sin, tiempo_vuelo_sin);
    
    texto_info_con = sprintf(['Con Arrastre\nVelocidad Inicial: %.2f m/s\nÁngulo: %.2f grados\n' ...
        'Altura Máxima: %.2f m\nDistancia Máxima: %.2f m\nTiempo de Vuelo: %.2f s'], ...
        v0, rad2deg(theta), max(y_con), distancia_con, tiempo_vuelo_con);
    
    annotation('textbox', [0.15, 0.7, 0.25, 0.15], 'String', texto_info_sin, 'FitBoxToText', 'on', 'BackgroundColor', [0, 0.5, 1], 'Color', 'white');
    annotation('textbox', [0.6, 0.7, 0.25, 0.15], 'String', texto_info_con, 'FitBoxToText', 'on', 'BackgroundColor', [1, 0.5, 0], 'Color', 'white');
    
    hold off;
end

function [x, y, distancia_maxima, tiempo_vuelo] = simulacionSinArrastre(theta, v0, m, g, tiempo_final, dt)
    % Inicialización de variables
    vx = v0 * cos(theta);  
    vy = v0 * sin(theta);
    num_pasos = ceil(tiempo_final / dt);
    x = zeros(1, num_pasos);
    y = zeros(1, num_pasos);
    
    % Simulación sin arrastre
    for i = 2:num_pasos
        ax = 0; % No hay aceleración en x
        ay = -g; % Solo la gravedad afecta en y
        
        % Método de Runge-Kutta
        vx = vx + ax * dt;
        vy = vy + ay * dt;
        
        x(i) = x(i-1) + vx * dt;
        y(i) = y(i-1) + vy * dt;
        
        if y(i) <= 0
            tiempo_vuelo = (i - 1) * dt;
            distancia_maxima = x(i);
            break;
        end
    end
    
    if y(end) > 0
        tiempo_vuelo = tiempo_final;
        distancia_maxima = max(x);
    end
end

function [x, y, distancia_maxima, tiempo_vuelo] = simulacionConArrastre(theta, v0, m, r, A, g, tiempo_final, dt)
    rho = 1.225; % densidad del aire (kg/m^3)
    Cd = 0.5; % coeficiente de arrastre para una esfera
    turbulencia = 0.1; % factor de turbulencia

    % Inicialización de variables
    vx = v0 * cos(theta);  
    vy = v0 * sin(theta);
    num_pasos = ceil(tiempo_final / dt);
    x = zeros(1, num_pasos);
    y = zeros(1, num_pasos);
    
    % Simulación con arrastre
    for i = 2:num_pasos
        velocidad_aire = sqrt(vx^2 + vy^2);
        vrelx = vx;
        vrely = vy - turbulencia * velocidad_aire;
        
        Fd = 0.5 * rho * velocidad_aire^2 * Cd * A;
        Fdx = -Fd * (vrelx / velocidad_aire);
        Fdy = -Fd * (vrely / velocidad_aire);
        
        ax = Fdx / m;
        ay = (Fdy / m) - g;
        
        % Método de Runge-Kutta
        vx = vx + ax * dt;
        vy = vy + ay * dt;
        
        x(i) = x(i-1) + vx * dt;
        y(i) = y(i-1) + vy * dt;
        
        if y(i) <= 0
            tiempo_vuelo = (i - 1) * dt;
            distancia_maxima = x(i);
            break;
        end
    end
    
    if y(end) > 0
        tiempo_vuelo = tiempo_final;
        distancia_maxima = max(x);
    end
end