function Center_generation

% This function is used to generate the geometric centers of the obstacles.

warning off

delete('Center');
fid = fopen('Center', 'w');

global Polygon_obstacle_vertex
number_of_obstacles = length(Polygon_obstacle_vertex) / 8;

% Calculate the geometric center of the obstacles
for ii = 1 : number_of_obstacles
    temp = Polygon_obstacle_vertex(((ii-1)*8+1) : ((ii-1)*8+8));
    center_x = mean([temp(1),temp(3),temp(5),temp(7)]);
    center_y = mean([temp(2),temp(4),temp(6),temp(8)]);
    fprintf(fid,'%g 1 %f \r\n', ii, center_x);
    fprintf(fid,'%g 2 %f \r\n', ii, center_y);
end
fclose(fid);
