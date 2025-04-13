clear all;
close all;
clc;
cd('/home/vigno/ws_new/Assignment_V')

%% PART 1: simulation

clear all;
%out = sim('Assignment_V_Simulink.slx');
%save('data.mat');

%% PART 2: plotting travelled distance of vehicles/gate motion in time

load('data.mat');
t_sim = out.tout;
vehicles = out.position.Data;
theta = out.motion.Data; % Angoli in gradi
omega = out.velocity.Data; % Velocità in gradi/s
alpha = out.acceleration.Data; % Accelerazione in gradi/s²
car_waiting = out.car_waiting.Data;
car_passed = out.car_passed.Data;

figure()

% Car position (Vehicles motion)
subplot(4, 1, 1)
hold on
plot(t_sim, vehicles, 'b-', 'LineWidth', 1.5)
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$x[m]$', 'Interpreter', 'latex')
title('Car Position', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
% Add vertical lines for car_waiting and car_passed
for ii = 1:length(car_waiting)
    if car_waiting(ii) == 1
        xline(t_sim(ii), '--', 'Color', [0 1 1],  'LineWidth', 1.5); % Cyan dashed line for car_waiting
    end
    if car_passed(ii) == 1
        xline(t_sim(ii), '--', 'Color', [1 0.5 0],  'LineWidth', 1.5); % Orange dashed line for car_passed
    end
end
hold off

% Theta (Gate motion)
subplot(4, 1, 2)
hold on
plot(t_sim, theta, 'r-', 'LineWidth', 1.5)
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\theta\ [^\circ]$', 'Interpreter', 'latex')
title('Gate angle', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
% Add vertical lines
for ii = 1:length(car_waiting)
    if car_waiting(ii) == 1
        xline(t_sim(ii), '--', 'Color', [0 1 1],  'LineWidth', 1.5); % Cyan dashed line for car_waiting
    end
    if car_passed(ii) == 1
        xline(t_sim(ii), '--', 'Color', [1 0.5 0],  'LineWidth', 1.5); % Orange dashed line for car_passed
    end
end
hold off

% Omega (Velocity)
subplot(4, 1, 3)
hold on
plot(t_sim, omega, 'm-', 'LineWidth', 1.5)
ylim([-5,5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\omega\ [^\circ/\mathrm{s}]$', 'Interpreter', 'latex') % Velocità in gradi/s
title('Gate speed', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
% Add vertical lines
for ii = 1:length(car_waiting)
    if car_waiting(ii) == 1
        xline(t_sim(ii), '--', 'Color', [0 1 1],  'LineWidth', 1.5); % Cyan dashed line for car_waiting
    end
    if car_passed(ii) == 1
        xline(t_sim(ii), '--', 'Color', [1 0.5 0],  'LineWidth', 1.5); % Orange dashed line for car_passed
    end
end
hold off

% Alpha (Acceleration)
subplot(4, 1, 4)
hold on
plot(t_sim, alpha, 'g-', 'LineWidth', 1.5)
ylim([-1.5,1.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\alpha\ [^\circ/\mathrm{s}^2]$', 'Interpreter', 'latex') % Accelerazione in gradi/s²
title('Gate acceleration', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
% Add vertical lines
for ii = 1:length(car_waiting)
    if car_waiting(ii) == 1
        xline(t_sim(ii), '--', 'Color', [0 1 1], 'LineWidth', 1.5); % Cyan dashed line for car_waiting
    end
    if car_passed(ii) == 1
        xline(t_sim(ii), '--', 'Color', [1 0.5 0], 'LineWidth', 1.5); % Orange dashed line for car_passed
    end
end
hold off

%% PART 3: graphs for raise and lower

close all

max_acc_dec = 1;
max_speed = 4;

custom_raise = @(time) ...
    (time < 4) .* (0.5 * max_acc_dec .* time.^2) + ...
    (time >= 4 & time < 22.5) .* (8 + max_speed .* (time - 4)) + ...
    (time >= 22.5 & time <= 26.5) .* (82 + max_speed .* (time - 22.5) - 0.5 * max_acc_dec .* (time - 22.5).^2);

custom_raise_d1 = @(time) ...
    (time < 4) .* (max_acc_dec .* time) + ...
    (time >= 4 & time < 22.5) .* max_speed + ...
    (time >= 22.5 & time <= 26.5) .* (max_speed - max_acc_dec .* (time - 22.5));

custom_raise_d2 = @(time) ...
    (time < 4) .* max_acc_dec + ...
    (time >= 4 & time < 22.5) .* 0 + ...
    (time >= 22.5 & time <= 26.5) .* (-max_acc_dec);

custom_lower = @(time) ...
    (time < 4) .* (90 - 0.5 * max_acc_dec .* time.^2) + ...
    (time >= 4 & time < 22.5) .* (90 - 8 - max_speed .* (time - 4)) + ...
    (time >= 22.5 & time <= 26.5) .* (8 - max_speed .* (time - 22.5) + 0.5 * max_acc_dec .* (time - 22.5).^2);

custom_lower_d1 = @(time) ...
    (time < 4) .* (-max_acc_dec .* time) + ...
    (time >= 4 & time < 22.5) .* (-max_speed) + ...
    (time >= 22.5 & time <= 26.5) .* (-max_speed + max_acc_dec .* (time - 22.5));

custom_lower_d2 = @(time) ...
    (time < 4) .* (-max_acc_dec) + ...
    (time >= 4 & time < 22.5) .* 0 + ...
    (time >= 22.5 & time <= 26.5) .* max_acc_dec;

time=linspace(0,26.5,10000);

% Raising
figure()
subplot(3, 1, 1)
hold on
plot(time, custom_raise(time), 'b-', 'LineWidth', 1.5)
ylim([0,100])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\theta\ [^\circ]$', 'Interpreter', 'latex')
title('Gate angle (opening)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off

subplot(3, 1, 2)
hold on
plot(time, custom_raise_d1(time), 'r-', 'LineWidth', 1.5)
ylim([-1, 5])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\omega\ [^\circ/\mathrm{s}]$', 'Interpreter', 'latex')
title('Gate speed (opening)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off

subplot(3, 1, 3)
hold on
plot(time, custom_raise_d2(time), 'm-', 'LineWidth', 1.5)
ylim([-1.5,1.5])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\alpha\ [^\circ/\mathrm{s}^2]$', 'Interpreter', 'latex') % Velocità in gradi/s
title('Gate acceleration (opening)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off


figure()
% Lower
subplot(3, 1, 1)
hold on
plot(time, custom_lower(time), 'b-', 'LineWidth', 1.5)
ylim([0,100])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\theta\ [^\circ]$', 'Interpreter', 'latex')
title('Gate angle (lowering)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off

subplot(3, 1, 2)
hold on
plot(time, custom_lower_d1(time), 'r-', 'LineWidth', 1.5)
ylim([-5,1])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\omega\ [^\circ/\mathrm{s}]$', 'Interpreter', 'latex')
title('Gate speed (lowering)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off

subplot(3, 1, 3)
hold on
plot(time, custom_lower_d2(time), 'm-', 'LineWidth', 1.5)
ylim([-1.5,1.5])
xlim([0,26.5])
grid on
box on
xlabel('Time [s]', 'Interpreter', 'latex')
ylabel('$\alpha\ [^\circ/\mathrm{s}^2]$', 'Interpreter', 'latex') % Velocità in gradi/s
title('Gate acceleration (lowering)', 'Interpreter', 'latex')
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on')
hold off