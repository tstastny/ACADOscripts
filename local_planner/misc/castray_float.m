function [x_occ, y_occ, h_occ, occ_detected, tri, d_occ, p_occ, p1, p2, p3, ii, t_init, t1, t2 ,t3] = ...
    castray_float(r0, r1, v, terr_dis, terr_mat, len_x, len_y, output_everything)
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
%
% INPUTS:
%
% (float) r0[3]                 start position (n,e,d) [m]
% (float) r1[3]                 end position (n,e,d) [m]
% (float) v[3]                  ray unit vector (n,e,d)
% (float) terr_dis              terrain discretization
% (float) terr_mat[len_y,len_x] elevation map
% (int)   len_x                 length of terrain map x axis
% (int)   len_y                 length of terrain map y axis
% (bool)  output_everything     outputs all visited cells if true, only the
%                               cell containing an occlusion if false
%
% OUTPUTS:
%
% (int)   x_occ[<n]             list of x coordinates of visited cells
% (int)   y_occ[<n]             list of y coordinates of visited cells
% (bool)  occ_detected          true if occlusion detected
% (int)   tri                   occluding triangle type: 0 = BR, 1 = TL
% (float) d_occ                 distance to the ray-triangle intersection [m]
% (float) p_occ[3]              coord. of the ray-triangle intersection [m]
% (float) p1[3]                 coord. of triangle vertex 1 (e,n,u) [m]
% (float) p2[3]                 coord. of triangle vertex 2 (e,n,u) [m]
% (float) p3[3]                 coord. of triangle vertex 3 (e,n,u) [m]
% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

tic;

% initialize
ret = false;
tri = 0;
d_occ = 0;
p_occ = zeros(3,1);
p1 = zeros(3,1);
p2 = zeros(3,1);
p3 = zeros(3,1);

% convert to enu
v = [v(2);v(1);-v(3)];

% relative start position
x0 = r0(2)/terr_dis;
y0 = r0(1)/terr_dis;

% initial height
h0 = -r0(3);

% relative end position
x1 = r1(2)/terr_dis;
y1 = r1(1)/terr_dis;

% end height
h1 = -r1(3);

% line deltas
dx = abs(x1 - x0);
dy = abs(y1 - y0);

% initial cell origin
x = floor(x0);
y = floor(y0);

% unit change in line length per x/y
dt_dx = 1.0 / dx;
dt_dy = 1.0 / dy;

% change in height per unit line length (t)
dh = h1 - h0;

% number of cells we pass through
n = floor(x1)-x + floor(y1)-y + 1;

% allocate lists
x_occ = zeros(n,1);
y_occ = zeros(n,1);
h_occ = zeros(n,1);

if (dx == 0)
    x_inc = 0;
    t_next_horizontal = dt_dx; % infinity
    t_last_horizontal = dt_dx; % infinity
elseif (x1 > x0)
    x_inc = 1;
    t_next_horizontal = (x + 1 - x0) * dt_dx; % remember x is "floor(x0)" here
    t_last_horizontal = (x0 - x) * dt_dx;
else
    x_inc = -1;
    t_next_horizontal = (x0 - x) * dt_dx; % remember x is "floor(x0)" here
    t_last_horizontal = (x + 1 - x0) * dt_dx;
end

if (dy == 0)
    y_inc = 0;
    t_next_vertical = dt_dy; % infinity
    t_last_vertical = dt_dy; % infinity
elseif (y1 > y0)
    y_inc = 1;
    t_next_vertical = (y + 1 - y0) * dt_dy; % remember y is "floor(y0)" here
    t_last_vertical = (y0 - y) * dt_dy;
else
    y_inc = -1;
    t_next_vertical = (y0 - y) * dt_dy; % remember y is "floor(y0)" here
    t_last_vertical = (y + 1 - y0) * dt_dy;
end

% find intersection in opposite direction to initialize cell entrance
% condition
last_step_was_vert = (t_last_vertical < t_last_horizontal);

% initialize entrance height
h_entr = h0;

t_init = toc;

t1 = zeros(n,1);
t2 = zeros(n,1);
t3 = zeros(n,1);

occ_detected = false;
for ii = 1:n
    
    tic;
    
    % bound corner coordinates
    x_check = max(min(x,len_x-1),0);
    y_check = max(min(y,len_y-1),0);
    x_check1 = max(min(x_check+1,len_x-1),0); 
    y_check1 = max(min(y_check+1,len_y-1),0); 
    
    % check the next step we will take and compute the exit height
    if (t_next_vertical < t_next_horizontal)
        % next step is vertical
        take_vert_step = true;
        t = t_next_vertical; % current step
        t_next_vertical = t_next_vertical + dt_dy;
    else
        % next step is horizontal
        take_vert_step = false;
        t = t_next_horizontal; % current step
        t_next_horizontal = t_next_horizontal + dt_dx;
    end
    
    % take minimum of entrance and exit height for check
    h_exit = h0 + dh * t;
    if dh>0
        h_check = h_entr;
    else
        h_check = h_exit;
    end
    h_entr = h_exit;
    h_occ(ii) = h_check;
    
    t1(ii) = toc;
    tic;
    
    % check cell triangles
    if last_step_was_vert % / / / / / / / / / / / / / / / / / / / / / / / /
        % vertical entrance step

        if take_vert_step
            % next step is vertical

            if y_inc > 0
                % BR, TL

                % check bottom-right triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end

                % check top-left triangle corners
%                     check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
%                     check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3) && ~ret
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end
            else
                % TL, BR

                % check top-left triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end

                % check bottom-right triangle corners
%                     check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
%                     check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3) && ~ret
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end
            end

        else %  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            % next step is horizontal

            if (y_inc > 0 && x_inc > 0)
                % BR

                % check bottom-right triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end
            elseif (y_inc < 0 && x_inc < 0)
                % TL

                % check top-left triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end
            else
                % BR, TL

                % check bottom-right triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end

                % check top-left triangle corners
%                     check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
%                     check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3) && ~ret
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end
            end
        end
    else % last step was horizontal / / / / / / / / / / / / / / / / / / / /
        if take_vert_step
            % next step is vertical

            if (x_inc > 0)
                % TL

                % check top-left triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end

                if (y_inc < 0) && ~ret
                    % BR

                    % check bottom-right triangle corners
%                     check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                    check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
%                     check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                    if (check1 || check4 || check3)
                        % check for ray-triangle intersection

                        p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                        p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                        p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                        [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);

                        if ret, tri = 0; end;
                    end
                end
            else
                % BR

                % check bottom-right triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end

                if (y > 0) && ~ret
                    % TL

                    % check top-left triangle corners
%                     check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                    check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
%                     check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                    if (check1 || check2 || check3)
                        % check for ray-triangle intersection

                        p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                        p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                        p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                        [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);

                        if ret, tri = 1; end;
                    end
                end
            end
        else %  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            % next step is horizontal

            if x_inc > 0
                % TL, BR

                % check top-left triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end

                % check bottom-right triangle corners
%                 check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
%                 check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3) && ~ret
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end
            else
                % BR, TL

                % check bottom-right triangle corners
                check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check4 = terr_mat(y_check+1,x_check1+1) > h_check; % corner 4 (bottom right) 
                check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check4 || check3)
                    % check for ray-triangle intersection

                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x+1); terr_dis*(y); terr_mat(y_check+1,x_check1+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 0; end;
                end

                % check top-left triangle corners
%                 check1 = terr_mat(y_check+1,x_check+1) > h_check; % corner 1 (bottom left)
                check2 = terr_mat(y_check1+1,x_check+1) > h_check; % corner 2 (top left)
%                 check3 = terr_mat(y_check1+1,x_check1+1) > h_check; % corner 3 (top right)

                if (check1 || check2 || check3) && ~ret
                    % check for ray-triangle intersection
                    
                    p1 = [terr_dis*(x); terr_dis*(y); terr_mat(y_check+1,x_check+1)];
                    p2 = [terr_dis*(x); terr_dis*(y+1); terr_mat(y_check1+1,x_check+1)];
                    p3 = [terr_dis*(x+1); terr_dis*(y+1); terr_mat(y_check1+1,x_check1+1)];

                    [ret, d_occ, p_occ] = intersect_triangle([r0(2); r0(1); h0], v, p1, p2, p3);
                    
                    if ret, tri = 1; end;
                end
            end
        end
    end % / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
    
    t2(ii) = toc;
    tic;
    
    % update lists / return if occlusion detected
    if ret
        x_occ(ii) = x;
        y_occ(ii) = y;
        occ_detected = true;
        return;
    elseif output_everything
        x_occ(ii) = x;
        y_occ(ii) = y;
    end
    
    % actually take the step
    if take_vert_step % (t_next_vertical < t_next_horizontal)
        % take a vertical step
        y = y + y_inc;
    else
        % take a horizontal step
        x = x + x_inc;
    end
    
    last_step_was_vert = take_vert_step;
    
    t3(ii) = toc;
end