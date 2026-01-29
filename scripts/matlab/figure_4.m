%% ATLAS-T Swarm Combined Analysis Figure Generation
% "Make it more good way" - Optimized for Single High-Res Publication Figure
% Combines Tension Analysis (A) and Path Consistency (B)
% Author: Chandan Sheikder
% Date: 2026

clear; close all; clc;

% --- Global Style Settings ---
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultTextFontName', 'Arial');
set(0, 'DefaultAxesFontSize', 9);
set(0, 'DefaultLineLineWidth', 1.2);
set(0, 'DefaultFigureColor', 'w');

% Define Color Palettes (Colorblind safe / Professional)
colors_tension = lines(7); 
colors_path_s1 = [0, 0.4470, 0.7410]; % Muted Blue
colors_path_s2 = [0.8500, 0.3250, 0.0980]; % Muted Orange

%% ========================================================================
%  FIGURE 1: Combined Swarm Analysis
% ========================================================================
% Tall Portrait Aspect Ratio for 2-Stacked Analysis
fig1 = figure('Name', 'Combined Swarm Analysis', ...
              'Position', [100, 50, 900, 1100]); 

% Create 4x2 Layout (Rows 1-2: Tension, Rows 3-4: Path)
t = tiledlayout(fig1, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(t, 'ATLAS-T Swarm Performance Analysis', 'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  SECTION A: Tension Distribution (Top Half)
% ========================================================================

% --- Data Generation (Tension) ---
time = 0:0.1:20;
num_robots = 8;
base_signal = sin(time/2) + 2.5; 
tensions = zeros(length(time), num_robots);
for i = 1:num_robots
    tensions(:, i) = base_signal + 0.15*randn(size(time)) + (0.3*rand);
end

% A.1 Time-Series Plot
nexttile(t);
hold on;
for i = 1:num_robots
    p = plot(time, tensions(:, i), 'Color', [colors_tension(1,:), 0.3]); % Transparent blue
end
plot(time, mean(tensions, 2), 'k-', 'LineWidth', 1.5); % Mean line black
yline(2.5, 'r--', 'Nominal', 'LabelHorizontalAlignment', 'left');
hold off;
title('A.1 Individual Robot Tensions');
ylabel('Tension (N)');
xlim([0, 20]); grid on; box on;

% A.2 Box Plot (Scenario Comparison)
nexttile(t);
scen1 = 2 + 0.4*randn(100,1);
scen2 = 4 + 1.1*randn(100,1);
scen3 = 2.2 + 0.3*randn(100,1); % Added a 3rd scenario for richness
g_data = [scen1; scen2; scen3];
g_label = [repmat({'Static'},100,1); repmat({'Dynamic'},100,1); repmat({'Recov.'},100,1)];
b = boxchart(categorical(g_label), g_data);
b.BoxFaceColor = colors_tension(2,:);
b.MarkerStyle = '.';
title('A.2 Tension by Operation Mode');
grid on; box on;

% A.3 Heatmap (Correlation)
nexttile(t);
% Generate slightly correlated matrix
c_mat = corrcoef(tensions) .* (ones(num_robots) - 0.2*eye(num_robots)); 
x_lbls = arrayfun(@(x) sprintf('R%d',x), 1:num_robots, 'UniformOutput', false);
h = heatmap(x_lbls, x_lbls, c_mat, 'ColorbarVisible', 'off');
h.Title = 'A.3 Tension Correlation';
h.Colormap = sky;
h.ColorLimits = [0.5, 1];

% A.4 Bar Chart (State Distribution)
nexttile(t);
states = {'Pull', 'Slack', 'Hold', 'Error'};
vals = [55, 30, 12, 3];
bb = bar(categorical(states, states), vals);
bb.FaceColor = 'flat';
bb.CData = repmat(colors_tension(3,:), 4, 1);
bb.CData(4,:) = [0.8 0.2 0.2]; % Red for error
title('A.4 Tension State Frequency');
ylabel('freq (%)'); 
ylim([0 70]); grid on; box on;

%% ========================================================================
%  SECTION B: Path Consistency (Bottom Half)
% ========================================================================

% --- Data Generation (Paths) ---
theta = linspace(0, 2*pi, 80);
% Scenario 1: Tight
x1_base = cos(theta); y1_base = sin(theta);
% Scenario 2: Loose
x2_base = 1.5*cos(theta); y2_base = 1.5*sin(theta);

% B.1 Trajectories Scenario 1
nexttile(t);
hold on; axis equal;
for i = 1:8
    noise_x = 0.05*randn(size(theta)) + i*0.1;
    noise_y = 0.05*randn(size(theta));
    plot(x1_base + noise_x, y1_base + noise_y, 'Color', [colors_path_s1, 0.5]);
end
title('B.1 Trajectory: Stable Swarm');
xlabel('X (m)'); ylabel('Y (m)');
grid on; box on;

% B.2 Trajectories Scenario 2
nexttile(t);
hold on; axis equal;
for i = 1:8
    noise_x = 0.15*randn(size(theta)) + i*0.1;
    noise_y = 0.15*randn(size(theta)) + 0.2*sin(3*theta); % Waviness
    plot(x2_base + noise_x, y2_base + noise_y, 'Color', [colors_path_s2, 0.5]);
end
title('B.2 Trajectory: Perturbed Swarm');
xlabel('X (m)'); ylabel('Y (m)');
grid on; box on;

% B.3 Statistical Bar Chart
nexttile(t);
mu = [5.90, 10.97];
sigma = [0.74, 1.28];
cats = categorical({'Stable', 'Perturbed'});
cats = reordercats(cats, {'Stable', 'Perturbed'});
b3 = bar(cats, mu);
b3.FaceColor = 'flat';
b3.CData(1,:) = colors_path_s1;
b3.CData(2,:) = colors_path_s2;
hold on;
errorbar(cats, mu, sigma, 'k.', 'LineWidth', 1.5);
hold off;
title('B.3 Mean Path Length (\mu \pm \sigma)');
ylabel('Length (m)');
grid on;

% B.4 Efficiency Box Plot
nexttile(t);
eff_s1 = 1.0 + abs(0.1*randn(30,1));
eff_s2 = 1.4 + abs(0.3*randn(30,1));
group_eff = [eff_s1; eff_s2];
cat_eff = [repmat({'Stable'},30,1); repmat({'Perturbed'},30,1)];
bc = boxchart(categorical(cat_eff), group_eff);
bc.BoxFaceColor = [0.5 0.5 0.5];
yline(1, 'g--', 'Optimal');
title('B.4 Path Efficiency Ratio');
ylabel('Ratio (L / L_{opt})');
grid on; box on;

%% ========================================================================
%  VISUAL SEPARATION & ANNOTATION
% ========================================================================

% Add invisible axes to place section headers if standard titles aren't enough
% or create a visual line separator
annotation(fig1, 'line', [0.1 0.9], [0.5 0.5], 'Color', [0.5 0.5 0.5], 'LineStyle', ':');

% Add Section Headers on the side using Annotation TextArrow (invisible arrow)
% Section A Label
annotation(fig1, 'textbox', [0.01, 0.9, 0.1, 0.05], 'String', 'A', ...
    'EdgeColor', 'none', 'FontSize', 24, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);

% Section B Label
annotation(fig1, 'textbox', [0.01, 0.43, 0.1, 0.05], 'String', 'B', ...
    'EdgeColor', 'none', 'FontSize', 24, 'FontWeight', 'bold', 'Color', [0.2 0.2 0.2]);

%% Export
fprintf('Figure generated.\n');
% Uncomment below to save
exportgraphics(fig1, 'Combined_Swarm_Analysis.png', 'Resolution', 300);