%% ATLAS-T Swarm: Supplementary Figure S8 Generation
% Topic: Framework Performance Across Environments (Table 18)
% Style: Nature/Scientific Publication (Clean, High Contrast)
% Author: Chandan Sheikder
% Date: 2026

clear; close all; clc;

% --- Global Style Settings ---
set(0, 'DefaultFigureColor', 'w');
set(0, 'DefaultAxesColor', 'w');
set(0, 'DefaultAxesFontName', 'Arial');
set(0, 'DefaultTextFontName', 'Arial');
set(0, 'DefaultAxesFontSize', 9);
set(0, 'DefaultLineLineWidth', 1.0);

% --- Data Setup (From Table 18) ---
categories = {'Nuclear Facility', 'Subsea Pipeline', 'Urban Disaster', 'Indoor Farm'};
% Data: [Nuclear, Subsea, Urban, Indoor]
data_plr = [88, 85, 78, 94];          % Path Efficiency (%)
data_col = [0.20, 0.28, 0.35, 0.10];  % Collision Rate (#/m)
data_ent = [0.03, 0.08, 0.12, 0.01];  % Entanglement Risk (#/hr)

% --- Semantic Color Palette ---
% Distinct colors for each environment to allow easy tracking across panels
% 1. Nuclear: Slate/Grey-Blue
% 2. Subsea: Deep Ocean Blue
% 3. Urban: Burnt Orange (Danger/Chaos)
% 4. Farm: Leaf Green (Safe/Growth)
env_colors = [
    0.45 0.50 0.60;  % Nuclear
    0.00 0.45 0.74;  % Subsea
    0.85 0.33 0.10;  % Urban
    0.47 0.67 0.19   % Farm
];

%% ========================================================================
%  FIGURE GENERATION
% ========================================================================
figS8 = figure('Name', 'Fig S8: Environmental Performance', ...
               'Position', [100, 100, 1200, 450]); % Wide aspect ratio

% 1 Row, 3 Columns
t = tiledlayout(figS8, 1, 3, 'TileSpacing', 'compact', 'Padding', 'normal');

% Main Title
title(t, 'Supplementary Fig. S8 | Framework Performance Across Environments', ...
    'FontSize', 12, 'FontWeight', 'bold');

% -------------------------------------------------------------------------
% Panel A: Path Efficiency (PLR)
% -------------------------------------------------------------------------
ax1 = nexttile(t);
hold on;
b1 = bar(1:4, data_plr, 0.6, 'FaceColor', 'flat');
b1.CData = env_colors; % Apply colors

% Add Value Labels on top of bars
for i = 1:4
    text(i, data_plr(i) + 2, sprintf('%d%%', data_plr(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold');
end

ylabel('Path Efficiency (PLR %)', 'FontWeight', 'bold');
ylim([0 110]); % Give headroom for text
yticks(0:20:100);
title('Path Efficiency (Higher is Better)', 'FontWeight', 'normal', 'FontSize', 10);

% Panel Label 'a'
text(-0.15, 1.05, 'a', 'Units', 'normalized', 'FontSize', 14, 'FontWeight', 'bold');

setup_axis(ax1, categories);

% -------------------------------------------------------------------------
% Panel B: Collision Rate
% -------------------------------------------------------------------------
ax2 = nexttile(t);
hold on;
b2 = bar(1:4, data_col, 0.6, 'FaceColor', 'flat');
b2.CData = env_colors;

% Add Value Labels
for i = 1:4
    text(i, data_col(i) + 0.01, sprintf('%.2f', data_col(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end

ylabel('Collision Rate (#/m)', 'FontWeight', 'bold');
ylim([0 0.45]);
title('Collision Rate (Lower is Better)', 'FontWeight', 'normal', 'FontSize', 10);

% Panel Label 'b'
text(-0.15, 1.05, 'b', 'Units', 'normalized', 'FontSize', 14, 'FontWeight', 'bold');

setup_axis(ax2, categories);

% -------------------------------------------------------------------------
% Panel C: Entanglement Risk
% -------------------------------------------------------------------------
ax3 = nexttile(t);
hold on;
b3 = bar(1:4, data_ent, 0.6, 'FaceColor', 'flat');
b3.CData = env_colors;

% Add Value Labels
for i = 1:4
    text(i, data_ent(i) + 0.005, sprintf('%.2f', data_ent(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 8);
end

ylabel('Entanglement Risk (#/hr)', 'FontWeight', 'bold');
ylim([0 0.15]);
title('Entanglement Risk (Lower is Better)', 'FontWeight', 'normal', 'FontSize', 10);

% Panel Label 'c'
text(-0.15, 1.05, 'c', 'Units', 'normalized', 'FontSize', 14, 'FontWeight', 'bold');

setup_axis(ax3, categories);


%% ========================================================================
%  HELPER FUNCTION FOR AXIS STYLING
% ========================================================================
function setup_axis(ax, cats)
    xticks(ax, 1:4);
    xticklabels(ax, cats);
    xtickangle(ax, 30); % Rotate for readability
    grid(ax, 'on');
    ax.YGrid = 'on';
    ax.XGrid = 'off'; % Vertical grid lines are distracting in bar charts
    ax.TickLength = [0 0]; % Hide tick marks
    box(ax, 'off'); % Open design (Nature style)
    
    % Draw a bottom line manually since box is off
    x_lims = xlim(ax);
    y_lims = ylim(ax);
    line(x_lims, [y_lims(1) y_lims(1)], 'Color', 'k', 'LineWidth', 1);
end

%% Export
fprintf('Supplementary Figure S8 generated.\n');
% exportgraphics(figS8, 'Supplementary_Fig_S8_EnvPerformance.png', 'Resolution', 600);