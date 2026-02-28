%% ATLAS-T Swarm: Supplementary Figure S7 (High-Contrast / Black Text)
% Topic: Fuzzy Logic Controller Rules & Membership
% Style: Nature/Science (Black Text, White Background, Clean Lines)
% Author: Chandan Sheikder
% Date: 2026

clear; close all; clc;

% --- Global Publication Style Settings (Strict Black/White Enforcement) ---
set(0, 'DefaultFigureColor', 'w');
set(0, 'DefaultAxesColor', 'w');
set(0, 'DefaultAxesFontName', 'Arial'); % Standard sans-serif for Figures
set(0, 'DefaultTextFontName', 'Arial');
set(0, 'DefaultAxesFontSize', 8);       % 8pt is standard for captions/labels
set(0, 'DefaultAxesLineWidth', 0.75);   % Thin, crisp axes
set(0, 'DefaultLineLineWidth', 1.5);
set(0, 'DefaultTextColor', [0 0 0]);    % FORCE BLACK TEXT
set(0, 'DefaultAxesXColor', [0 0 0]);   % FORCE BLACK AXES
set(0, 'DefaultAxesYColor', [0 0 0]);   % FORCE BLACK AXES
set(0, 'DefaultAxesZColor', [0 0 0]);   % FORCE BLACK AXES

% --- Scientific Color Palette (Data Lines Only) ---
% We keep color for the DATA lines for contrast, but text remains black.
col_blue  = [0 0.4470 0.7410];
col_yell  = [0.9290 0.6940 0.1250];
col_red   = [0.8500 0.3250 0.0980];
col_purp  = [0.4940 0.1840 0.5560];

%% ========================================================================
%  FIGURE SETUP
% ========================================================================
% 180mm width (Full page width)
figS7 = figure('Name', 'Fig S7: FLC Architecture', 'Position', [100, 100, 1000, 600]);
t = tiledlayout(figS7, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

%% ========================================================================
%  ROW 1: MEMBERSHIP FUNCTIONS (a, b, c)
% ========================================================================

% --- Panel (a): Obstacle Proximity ---
nexttile(t);
hold on;
x = 0:0.01:6;
% Membership Definitions
mf_near = max(0, min(1, (1.5 - x)/1.0));         
mf_med  = exp(-(x - 2.5).^2 / (2*0.5^2));        
mf_far  = 1 ./ (1 + exp(-3*(x-4)));              

% Plot Data Lines
plot(x, mf_near, 'Color', col_red, 'LineWidth', 1.5);
plot(x, mf_med,  'Color', col_yell, 'LineWidth', 1.5);
plot(x, mf_far,  'Color', col_blue, 'LineWidth', 1.5);

% Fill Critical area (Light Red)
area(x, mf_near, 'FaceColor', col_red, 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Annotations (STRICTLY BLACK TEXT)
text(0.2, 0.9, 'Near', 'Color', 'k', 'FontWeight', 'bold');
text(2.5, 0.9, 'Medium', 'Color', 'k', 'HorizontalAlignment','center');
text(5.0, 0.9, 'Far', 'Color', 'k');

% Formatting
xlabel('Distance (m)', 'Color', 'k'); 
ylabel('Membership (\mu)', 'Color', 'k');
ylim([0 1.1]); xlim([0 6]);
title('Obstacle Proximity', 'FontWeight', 'normal', 'Color', 'k');
% Panel Label
text(-0.15, 1.05, 'a', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k'); 
grid off; box on; 
hold off;

% --- Panel (b): Tether Tension ---
nexttile(t);
hold on;
x_t = 0:0.1:16;
% Definitions
mf_low  = max(0, min(1, (4 - x_t)/2));
mf_mod  = exp(-(x_t - 7).^2 / (2*1.2^2));
mf_high = exp(-(x_t - 11).^2 / (2*1.2^2));
mf_crit = 1 ./ (1 + exp(-2*(x_t-13)));

plot(x_t, mf_low,  'Color', col_blue);
plot(x_t, mf_mod,  'Color', col_yell);
plot(x_t, mf_high, 'Color', col_purp); 
plot(x_t, mf_crit, 'Color', col_red, 'LineWidth', 2);

% Fill Critical area
area(x_t(x_t>10), mf_crit(x_t>10), 'FaceColor', col_red, 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% Annotations (STRICTLY BLACK TEXT)
text(1, 0.9, 'Low', 'Color', 'k');
text(7, 0.9, 'Mod', 'Color', 'k', 'HorizontalAlignment','center');
text(14, 0.6, 'Critical', 'Color', 'k', 'FontWeight', 'bold');

xlabel('Tension (N)', 'Color', 'k'); 
ylim([0 1.1]); xlim([0 16]);
title('Tether Tension', 'FontWeight', 'normal', 'Color', 'k');
text(-0.15, 1.05, 'b', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
grid off; box on;
hold off;

% --- Panel (c): Goal Alignment ---
nexttile(t);
hold on;
x_a = -pi:0.05:pi;
mf_ali = exp(-(x_a).^2 / (2*0.5^2));

plot(x_a, mf_ali, 'Color', col_blue, 'LineWidth', 2);
area(x_a, mf_ali, 'FaceColor', col_blue, 'FaceAlpha', 0.1, 'EdgeColor', 'none');

xlabel('Heading Error (rad)', 'Color', 'k'); 
xticks([-3 0 3]); xticklabels({'-\pi', '0', '\pi'});
ylim([0 1.1]); xlim([-3.2 3.2]);

% Annotations (STRICTLY BLACK TEXT)
text(0, 0.9, 'Aligned', 'Color', 'k', 'HorizontalAlignment','center');

title('Goal Alignment', 'FontWeight', 'normal', 'Color', 'k');
text(-0.15, 1.05, 'c', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
grid off; box on;

%% ========================================================================
%  ROW 2: CONTROL SURFACE & TABLE (d, e)
% ========================================================================

% --- Panel (d): Control Surface (Spans 2 columns) ---
nexttile(t, 4, [1 2]);
[D, Tens] = meshgrid(0:0.1:6, 0:0.5:16);

% Logic: Corrective Action Calculation
Action = (1 ./ (1 + D)) + (Tens/16).^2; 
Action = Action ./ max(Action(:)); % Normalize

s = surf(D, Tens, Action);
s.EdgeColor = 'none';
s.FaceColor = 'interp';
colormap(parula); % Data color remains, but surrounding text is black
view(135, 30);
camlight; lighting gouraud;

hold on;
% Mark the specific rule point
p_d = 0.5; p_t = 14; 
p_z = (1/(1+p_d)) + (p_t/16)^2; p_z = p_z/max(Action(:));
plot3(p_d, p_t, p_z, 'o', 'MarkerSize', 8, 'MarkerFaceColor', col_red, 'MarkerEdgeColor', 'k');

% Annotation (STRICTLY BLACK TEXT)
text(p_d, p_t, p_z+0.25, 'Safety Override', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');

xlabel('Obs. Distance (m)', 'Color', 'k');
ylabel('Tension (N)', 'Color', 'k');
zlabel('Action Gain', 'Color', 'k');
title('FLC Safety Response Surface', 'FontWeight', 'normal', 'Color', 'k');
text(-0.08, 1.1, 'd', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
grid on;
ax = gca; ax.GridColor = [0.2 0.2 0.2]; ax.GridAlpha = 0.3; % Subtle grey grid

% --- Panel (e): Scientific Table (Rule Base) ---
ax_tab = nexttile(t);
axis off;

% Table Content
headers = {'Proximity', 'Tension', 'Action', 'Cert.'};
rows = {
    'Near',   'High',     'Steer + Slow', '70%'; ...
    'Medium', 'Low',      'Maintain',     '85%'; ...
    'Far',    'Mod.',     'Accel + Slack','90%'; ...
    'Near',   'Critical', 'STOP/Rev',     '99%'
};

% Draw Lines (Standard Scientific Table Style - Black Lines)
y_top = 0.9;
y_head = 0.8;
y_bot = 0.2;

line([0 1], [y_top y_top], 'Color', 'k', 'LineWidth', 1.5);   % Top thick
line([0 1], [y_head y_head], 'Color', 'k', 'LineWidth', 0.8); % Header thin
line([0 1], [y_bot y_bot], 'Color', 'k', 'LineWidth', 1.5);   % Bottom thick

% Column X Positions
x_pos = [0.05, 0.35, 0.60, 0.90];

% Place Headers (BLACK TEXT)
for i = 1:4
    text(x_pos(i), y_top - 0.05, headers{i}, 'Color', 'k', 'FontWeight', 'bold', ...
        'FontSize', 9, 'HorizontalAlignment', 'left');
end

% Place Data Rows
y_curr = y_head - 0.12;
for r = 1:4
    % Row Highlighting for Critical Rule
    weight = 'normal';
    if strcmp(rows{r,2}, 'Critical')
        % Use a very light GREY box for highlight, not color, to maintain B&W text compatibility
        patch([0 1 1 0], [y_curr-0.03 y_curr-0.03 y_curr+0.07 y_curr+0.07], ...
              [0.9 0.9 0.9], 'FaceAlpha', 1, 'EdgeColor', 'none');
        weight = 'bold';
    end
    
    % Text (STRICTLY BLACK)
    text(x_pos(1), y_curr, rows{r,1}, 'Color', 'k', 'FontWeight', weight);
    text(x_pos(2), y_curr, rows{r,2}, 'Color', 'k', 'FontWeight', weight);
    text(x_pos(3), y_curr, rows{r,3}, 'Color', 'k', 'FontSize', 8);
    text(x_pos(4), y_curr, rows{r,4}, 'Color', 'k');
    
    y_curr = y_curr - 0.12;
end

title('Table 15: Selected Rules', 'FontWeight', 'normal', 'Color', 'k');
text(-0.15, 1.05, 'e', 'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');

%% ========================================================================
%  FINAL RENDER NOTE
% ========================================================================
% To save in high resolution for publication:
% exportgraphics(figS7, 'FigS7_HighContrast.png', 'Resolution', 600);