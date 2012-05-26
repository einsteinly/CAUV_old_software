#! /usr/bin/env octave

% columns: ['sim t', 'sim x', 'sim y', 'sim theta', 'slam t', 'slam x', 'slam y', 'slam theta']

arg_list = argv ();
for i = 1:nargin
    printf("processing %s...\n", arg_list{i});
    data = csv2cell(arg_list{i})(2:end,:);
    sim_xy = cell2mat(data(:,[2,3]));
    slam_xy = cell2mat(data(:,[6,7]));
    figure(i)
    hold off
    plot(sim_xy(:,1), sim_xy(:,2), '1')
    hold on
    plot(slam_xy(:,1), slam_xy(:,2), '3')
    axis([-2 22 -2 2])
    xlabel('Metres East')
    ylabel('Metres North')
    title('Simulated vs Recovered Trajectory')
endfor
refresh

