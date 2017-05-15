function plotCostmapCostDecay(inscribed_r, circunscribed_r, inflation_r, csfactors)
    % generates a plot with the costmap cost similar to the one on the
    % wiki.
    % 
    % more info: http://wiki.ros.org/costmap_2d --> inflation layer
    %
    % example:
    % - plotCostmapCostDecay(0.4, 0.5,1)
    % - plotCostmapCostDecay(0.4, 0.5,[0.5 1 2 5 10])


    % default cost_scaling_factor(s)
    if nargin < 4
        csfactors = [0.5 1 2 5 10];
    end
    
    
    
    % Inscribed inflated obstacle cost
    % see costmap_2d::INSCRIBED_INFLATED_OBSTACLE fro the actual value
    cost_lethal        = 255;
    cost_colission     = 254;

    % plot_margin_factor
    zoom_out = 1.2; % 30%
    
    % collision cost
    x_lethal = [0 0.1 0.1 1]*inscribed_r;
    y_lethal = [cost_lethal cost_lethal cost_colission cost_colission];

    % free cost
    x_free = [1 zoom_out]*inflation_r;
    y_free = [0 0];
    
    % decay function
    n_points = 50;
    px = round(n_points*0.8);
    distance = linspace(inscribed_r,inflation_r,n_points);
    decay_fn = @(csf) exp(-1.0*csf*(distance - inscribed_r))*(cost_colission - 1);

    % - -   plot   - -
    figure;
    hold on;
    
    % axes
    h(1) = plot([0 zoom_out]*inflation_r,[0 0],'-k');
    h(2) = plot([0 0],[0 280],'-k');
    
    % limits
    h(3) = plot([1 1]*inscribed_r,[0 270],'-b');     text(inscribed_r,270,'\leftarrow inscribed\_radius');
    h(4) = plot([1 1]*circunscribed_r,[0 255],'-b'); text(circunscribed_r,250,'\leftarrow circunscribed\_radius');
    h(5) = plot([1 1]*inflation_r,[0 200],'-b');     text(inflation_r,190,'inscribed\_radius \rightarrow','HorizontalAlignment','right');
    
    
    % costs
    h(6) = plot(x_lethal,y_lethal,'-xr');
    h(7) = plot(x_free,y_free,'o-g');
    for k=1:length(csfactors)
        csf = csfactors(k);
        decay = decay_fn(csf);
        h(8) = plot(distance,decay,'--r');
        
        dist = distance(px);
        cost = decay(px);
        string = ['\leftarrow c = ', sprintf('%.2f',csf)];
        text(dist,cost,string)
    end
    
    hold off;
    title('\texttt{cost\_scaling\_factor} effect on Costmap2D inflation costs','interpreter','Latex','fontsize',14);
    xlabel('distance to obstacle [m]','interpreter','Latex','fontsize',12);
    ylabel('cell cost [byte]','interpreter','Latex','fontsize',12);
    legend(h([6 7 8]), 'collision', 'freespace', 'exponential decays');
    axis([0 zoom_out*inflation_r -20 280])
    