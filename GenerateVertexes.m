function GenerateVertexes(gamma)
% Input: scalar called \gamma, retunr void, but a new file is written as
% the output. By default, the obstacles are all assumed to be quadrilateral.
% If other types of polygonal obstacles are to be added, the .mod file also
% needs to be modified accordingly. This is not difficult.
%
% This function is used to generate the new vertexes according to the
% current setting of \gamma. When \gamma = 1, the new vertexes are idential
% to the ones on the original polygon(s); when \gamma = 0, all the new vertexes
% gather at the geometric center of the original polygon; when 0< \gamma <1,
% the new polygon appears to be a shrinked version of the original one.

if ((gamma > 1) || (gamma < 0))
    error 'Invalid \gamma';
end

warning off

% Load the locations of the vertexes on the original polygonal obstacles
global polygon_obstacle_vertex
number_of_obstacles = length(polygon_obstacle_vertex) / 8;

% Calculate the geometric center of the obstacles
center = [];
for ii = 1 : number_of_obstacles
    temp = polygon_obstacle_vertex(((ii-1)*8+1) : ((ii-1)*8+8));
    center_x = mean([temp(1),temp(3),temp(5),temp(7)]);
    center_y = mean([temp(2),temp(4),temp(6),temp(8)]);
    center = [center, repmat([center_x,center_y],1,4)];
end

Error_vector = polygon_obstacle_vertex - center;

% Calculation of the locations of the new vertexes according to the cureent
% setting of \gamma.
New_vertex = Error_vector .* gamma + center;

% Prepare to write a new file to store the locations of the new vertexes
delete('Current_vertex');
fid = fopen('Current_vertex', 'w');
ind = 0;
for ii = 1 : number_of_obstacles % From the first obstacle to the last
    for jj = 1 : 4 % From the first vertex to the fourth (the obstacles are all assumed to be quadrilateral)
        for kk = 1 : 2  % From the x coordinate to the y corordinate.
            ind = ind + 1;
            fprintf(fid,'%g %g %g %f \r\n', ii,jj,kk,New_vertex(ind));
        end
    end
end
fclose(fid);

delete('Number_obstacle');
fid = fopen('Number_obstacle', 'w');
fprintf(fid,'%g', number_of_obstacles);
fclose(fid);

Area = [];
for ii = 1 : number_of_obstacles
    temp = New_vertex(((ii-1)*8+1) : ((ii-1)*8+8));
    x_axis = [temp(1),temp(3),temp(5),temp(7)];
    y_axis = [temp(2),temp(4),temp(6),temp(8)];
    Area = [Area, CalculateArea(x_axis,y_axis)];
end

delete('Area');
fid = fopen('Area', 'w');
for ii = 1 : number_of_obstacles
    fprintf(fid,'%g    %g\r\n', ii, Area(ii));
end
fclose(fid);