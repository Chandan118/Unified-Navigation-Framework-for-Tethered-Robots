function create_combined_simulation_figure()
    % 1. SETUP FIGURE
    % Create a wide figure suitable for a document
    fig = figure('Name', 'Robot Navigation Analysis', 'Color', 'w', ...
                 'Position', [100, 100, 1200, 600]);
    
    % Use TiledLayout for modern, tight spacing
    t = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'normal');

    %% --- FIGURE A: SITE MAP (Left) ---
    ax1 = nexttile;
    hold(ax1, 'on');
    grid(ax1, 'on');
    box(ax1, 'on');
    
    % A. Draw Static Environment (Walls & Objects)
    % Outer Site Walls (Black Box)
    rectangle('Position', [-12 -12 24 24], 'LineWidth', 2, 'EdgeColor', 'k');
    
    % Machinery (Grey Boxes)
    draw_box([-9, 6, 4, 5], [0.6 0.6 0.6], 'Machinery');
    draw_box([-4, -10, 3, 8], [0.6 0.6 0.6], 'Machinery');
    draw_box([5, -8, 4, 6], [0.6 0.6 0.6], 'Machinery');
    
    % Pipes / Structural Data (Yellow)
    draw_box([-12, 11, 12, 1], [0.8 0.8 0.1], ''); % Top
    draw_box([9, -12, 1, 12], [0.8 0.8 0.1], 'Pipe'); % Right
    
    % Forklift (Red Box - Upright)
    draw_box([-9, 1, 2, 3], [1 0.3 0.3], 'Forklift');
    
    % Forklift (Red Box - Rotated 45 degrees)
    % We need to manually calculate patch coordinates for rotation
    cx = 1; cy = -6; w = 2; h = 3; angle = 45;
    R = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
    % Define corners relative to center
    pts = [-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2] * R';
    patch(ax1, cx + pts(:,1), cy + pts(:,2), [1 0.3 0.3], 'EdgeColor', 'k');
    text(cx+1.5, cy, 'Forklift', 'Color', [0.8 0 0], 'FontWeight', 'bold');

    % B. Draw Dynamic Agents
    % Spool
    plot(ax1, -10, -10.5, 's', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
    text(ax1, -9.5, -10.5, 'Spool');
    
    % Robot (Green Triangle)
    r_pos = [-2, -2];
    plot(ax1, r_pos(1), r_pos(2), '^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    text(ax1, r_pos(1), r_pos(2)-1.5, 'Robot', 'Color', [0 0.7 0], 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Tether (Magenta Dashed)
    plot(ax1, [-10, r_pos(1)], [-10.5, r_pos(2)], 'm--', 'LineWidth', 2);
    
    % Workers (Blue Crosses)
    plot(ax1, -8, -5, 'P', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    text(ax1, -7.5, -5, 'Worker', 'Color', 'b');
    plot(ax1, 8, 5, 'P', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    plot(ax1, 10, 9, 'P', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
    text(ax1, 10.5, 9, 'Worker', 'Color', 'b');

    % Simulated LiDAR Hits (Red dots scattered)
    % Random noise + clusters near walls
    lx = [rand(1,30)*24-12, -4+rand(1,10), 1+rand(1,10)];
    ly = [rand(1,30)*24-12, -5+rand(1,10), -6+rand(1,10)];
    plot(ax1, lx, ly, 'r.', 'MarkerSize', 2);

    % C. Formatting Figure A
    axis(ax1, 'equal');
    xlim(ax1, [-13 13]); ylim(ax1, [-13 13]);
    xlabel(ax1, 'X Coordinate (m)', 'FontWeight', 'bold');
    ylabel(ax1, 'Y Coordinate (m)', 'FontWeight', 'bold');
    title(ax1, '(a) Site Map & Agent Configuration', 'FontSize', 12);


    %% --- FIGURE B: HEATMAP & TRAJECTORY (Right) ---
    ax2 = nexttile;
    hold(ax2, 'on');
    
    % A. Generate Synthetic Heatmap Data
    [X, Y] = meshgrid(0:0.1:20, 0:0.1:20);
    Z = zeros(size(X));
    % Add gaussian blobs to look like the screenshot
    centers = [4 9; 11 11; 15 15; 18 5; 3 17; 8 13; 14 3; 18 12; 2 4];
    for i = 1:size(centers,1)
        Z = Z + exp(-((X-centers(i,1)).^2 + (Y-centers(i,2)).^2)./2.5);
    end
    Z = Z + 0.2*rand(size(Z)); % Slight noise

    % B. Plot Contours
    contourf(ax2, X, Y, Z, 12, 'LineColor', [0.3 0.3 0.3]);
    colormap(ax2, parula);
    
    % C. Draw the Curved Path (Spline)
    % Control points for the path
    px = [2, 5, 10, 14, 18, 18.5];
    py = [18, 14, 10, 15, 10, 5];
    % Smooth interpolation
    tt = 1:length(px);
    ts = linspace(1, length(px), 100);
    p_smooth_x = spline(tt, px, ts);
    p_smooth_y = spline(tt, py, ts);
    
    plot(ax2, p_smooth_x, p_smooth_y, 'r--', 'LineWidth', 2.5);
    
    % D. Anchor and Limits
    % Anchor Star
    plot(ax2, 2, 18, 'p', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'w');
    text(ax2, 2.5, 18.5, 'Anchor', 'Color', 'm', 'FontWeight', 'bold');
    
    % Range Arc (Grey)
    th = linspace(-0.5, 1.5, 100);
    r_arc = 17;
    plot(ax2, r_arc*cos(th)+2, r_arc*sin(th)+2, 'Color', [0.6 0.6 0.6], 'LineWidth', 3);
    
    % E. Formatting Figure B
    xlim(ax2, [0 20]); ylim(ax2, [0 20]);
    box(ax2, 'on');
    xlabel(ax2, 'X (m)'); ylabel(ax2, 'Y (m)');
    title(ax2, '(b) Probabilistic Field & Path Planning', 'FontSize', 12);
    
    %% --- GLOBAL TITLE ---
    sgtitle(fig, 'Multi-Modal Robot Localization & Mapping Results', ...
            'FontSize', 16, 'FontWeight', 'bold');
end

% --- Helper Function to Draw Rectangles ---
function draw_box(pos, col, lbl)
    rectangle('Position', pos, 'FaceColor', col, 'EdgeColor', 'k');
    if ~isempty(lbl)
        text(pos(1)+0.2, pos(2)+pos(4)-0.5, lbl, 'FontWeight', 'bold', 'FontSize', 8);
    end
end