function create_GA_Convergence_Figure_Fixed()
    % Clear environment
    close all; clc;
    
    % ---------------------------------------------------------
    % 1. DATA GENERATION
    % ---------------------------------------------------------
    
    % --- Panel A Data (Sigmoid simulation) ---
    gen = 0:50;
    % Sigmoid function for Best Fitness
    best_fitness = 0.4 + 0.48 ./ (1 + exp(-0.15 .* (gen - 20))); 
    
    % Average Fitness (lagging behind best)
    avg_fitness = best_fitness - (0.15 * exp(-0.02 * gen)); 
    
    % Create Min/Max bounds for shaded area
    max_val = best_fitness; 
    min_val = avg_fitness - 0.15;
    
    % --- Panel B Data (Mutation Sensitivity) ---
    mut_rates = [1, 5, 10]; 
    gens_converge = [75, 50, 35]; 
    final_fit_val = [0.82, 0.88, 0.78]; 

    % ---------------------------------------------------------
    % 2. FIGURE SETUP
    % ---------------------------------------------------------
    f = figure('Units', 'centimeters', 'Position', [2, 2, 20, 14]); 
    f.Color = 'w';
    
    % Create Tiled Layout
    t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(t, 'Supplementary Fig. S6: GA Optimization Convergence', 'FontWeight', 'bold', 'FontSize', 12);

    % ---------------------------------------------------------
    % 3. PANEL A: Fitness Convergence
    % ---------------------------------------------------------
    ax1 = nexttile;
    hold(ax1, 'on');
    
    % 1. Plot Shaded Range
    x_fill = [gen, fliplr(gen)];
    y_fill = [min_val, fliplr(max_val)];
    fill_color = [0.8, 0.9, 1]; 
    fill(x_fill, y_fill, fill_color, 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    
    % 2. Plot Lines
    pAvg = plot(gen, avg_fitness, 'k--', 'LineWidth', 1.5);
    blue_color = [0, 102/255, 204/255];
    pBest = plot(gen, best_fitness, '-', 'Color', blue_color, 'LineWidth', 2.5);
    
    % 3. Styling Panel A
    grid on;
    xlabel('Generations');
    ylabel('Fitness Value (0-1)');
    title('(A) Fitness convergence', 'FontWeight', 'bold');
    xlim([0 50]);
    ylim([0.1 1.0]);
    
    % 4. Annotations
    legend([pBest, pAvg], {'Best Fitness', 'Avg Fitness'}, 'Location', 'southeast', 'Box', 'off');
    
    % Arrow for Convergence
    text(30, 0.95, 'Convergence', 'HorizontalAlignment', 'center', 'FontSize', 9);
    quiver(30, 0.93, 13, -0.04, 0, 'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);
    
    % Text Box for Improvement
    text(5, 0.85, {'Fitness', 'Improvement:', '+120%'}, ...
        'EdgeColor', 'k', 'BackgroundColor', 'w', 'FontSize', 8);

    % ---------------------------------------------------------
    % 4. PANEL B: Mutation Rate Sensitivity
    % ---------------------------------------------------------
    ax2 = nexttile;
    
    % Dual Axis Setup
    yyaxis(ax2, 'left');
    
    % 1. Bar Chart
    b = bar(1:3, gens_converge, 0.5);
    b.FaceColor = 'flat';
    b.CData(1,:) = [0.8, 0.3, 0.3]; % Red
    b.CData(2,:) = [0.2, 0.7, 0.3]; % Green
    b.CData(3,:) = [0.3, 0.4, 0.8]; % Blue
    
    ylabel('Generations to Converge');
    ylim([0 90]);
    ax2.YColor = 'k'; 
    
    % 2. Line/Scatter Chart
    yyaxis(ax2, 'right');
    plot(1:3, final_fit_val, '-o', 'LineWidth', 2, 'MarkerSize', 8, ...
        'MarkerFaceColor', 'w', 'Color', [0.2, 0.2, 0.2]);
    ylabel('Final Fitness Score');
    ylim([0.7 0.95]);
    ax2.YColor = [0.2, 0.2, 0.2];
    
    % 3. Styling Panel B
    title('(B) Mutation rate impact', 'FontWeight', 'bold');
    xlabel('Mutation Rate (%)');
    set(ax2, 'XTick', 1:3, 'XTickLabel', {'1%', '5%', '10%'});
    grid on;
    
    % 4. Annotations for Panel B
    % CHANGED: Removed \star, used * symbol which is safer for default interpreter
    text(2, 0.94, '* Optimal Trade-off', 'Color', [0, 0.5, 0], ...
        'FontSize', 10, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    % Trade-off arrows
    text(1.1, 0.83, '\leftarrow Slow but decent', 'FontSize', 8, 'Color', 'r');
    text(2.4, 0.79, 'Fast but low quality \rightarrow', 'FontSize', 8, 'Color', 'b');

    % ---------------------------------------------------------
    % 5. ADDITIONAL TEXT ELEMENTS
    % ---------------------------------------------------------
    
    % --- GA Parameters Box ---
    paramStr = {
        '\bf GA Parameters:',
        'Pop Size: 100',
        'Selection: Tournament (3)',
        'Crossover: 0.8 (Single)',
        'Mutation: 0.05 (Gaussian)',
        'Chrom. Length: 8 params'
    };
    annotation('textbox', [0.1, 0.02, 0.25, 0.18], 'String', paramStr, ...
        'FitBoxToText', 'off', 'BackgroundColor', [0.95 0.95 0.95], 'FontSize', 8, ...
        'Interpreter', 'tex');

    % --- Chromosome Genes Box ---
    genesStr = {
        '\bf Chromosome (8):',
        'T_{min}, T_{max}, Slack_{thresh}',
        'Spool_{speed}, Reward_{scale}',
        'Weights: \omega_{LiDAR}, \omega_{IMU}, \omega_{US}'
    };
    annotation('textbox', [0.36, 0.02, 0.25, 0.15], 'String', genesStr, ...
        'FitBoxToText', 'off', 'BackgroundColor', 'none', 'EdgeColor', 'none', 'FontSize', 8, ...
        'Interpreter', 'tex');

    % --- Fitness Function Box ---
    % CHANGED: Added space in \Delta F to fix syntax error
    fitEq = {
        '\bf Fitness Function:',
        'F = 0.4\eta + 0.3(1-E_r) + 0.2C + 0.1E_{eff}',
        '',
        '\it Convergence Criteria:',
        '\Delta F < 1% over 5 gens' 
    };
    annotation('textbox', [0.62, 0.02, 0.3, 0.18], 'String', fitEq, ...
        'FitBoxToText', 'off', 'BackgroundColor', [0.95 0.95 0.95], 'FontSize', 8, ...
        'EdgeColor', 'black', 'Interpreter', 'tex');

    % ---------------------------------------------------------
    % 6. FONT STANDARDIZATION
    % ---------------------------------------------------------
    set(findall(gcf,'-property','FontName'),'FontName','Arial');
    
    disp('Figure created successfully without syntax errors.');
end