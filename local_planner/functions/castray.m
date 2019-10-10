function [occ_detected, d_occ, p_occ, p1, p2, p3] = castray(r0, r1, v, ...
    pos_n_origin, pos_e_origin, len_idx_n, len_idx_e, terr_dis, terr_map)

LEN_IDX_E = len_idx_e;
LEN_IDX_N = len_idx_n;
LEN_IDX_E_1 = LEN_IDX_E;
LEN_IDX_N_1 = LEN_IDX_N - 1;

% INPUTS: 
% 
% (double) r0[3]             	start position (e,n,u) [m] 
% (double) r1[3]             	end position (e,n,u) [m] 
% (double) v[3]              	ray unit vector (e,n,u) 
% (double) pos_n_origin        northing origin of terrain map 
% (double) pos_e_origin        easting origin of terrain map 
% (double) terr_dis          	terrain discretization 
% (double) terr_map          	terrain map 
% 
% OUTPUTS: 
% 
% (int)   occ_detected         0=no detection, 1=BR triangle detected, 2=TL triangle detected 
% (double) d_occ           	distance to the ray-triangle intersection [m] 
% (double) p_occ[3]        	coord. of the ray-triangle intersection [m] XXX: RELATIVE!
% (double) p1[3]           	coord. of triangle vertex 1 (e,n,u) [m] XXX: RELATIVE!
% (double) p2[3]             	coord. of triangle vertex 2 (e,n,u) [m] XXX: RELATIVE!
% (double)	p3[3]           	coord. of triangle vertex 3 (e,n,u) [m] XXX: RELATIVE!
%/  
 
% initialize 
occ_detected = 0;
p_occ = zeros(3,1);
d_occ = 0;
p1 = zeros(3,1);
p2 = zeros(3,1);
p3 = zeros(3,1);

% relative (unit) start position 
x0 = (r0(1) - pos_e_origin)/terr_dis; 
y0 = (r0(2) - pos_n_origin)/terr_dis; 
 
% initial height 
h0 = r0(3); 

% vector for triangle intersect inputs
r0_rel = [x0*terr_dis;y0*terr_dis;h0]; %XXX: this origin subtracting/adding is inefficient.. pick one and go with it for this function
 
% relative end position 
x1 = (r1(1) - pos_e_origin)/terr_dis; 
y1 = (r1(2) - pos_n_origin)/terr_dis; 
 
% end height 
h1 = r1(3); 
 
% line deltas 
dx = abs(x1 - x0); 
dy = abs(y1 - y0); 
 
% initial cell origin 
x = floor(x0); 
y = floor(y0); 

% change in height per unit line length (t) 
dh = h1 - h0; 
 
% number of cells we pass through 
n = abs(floor(x1)-x) + abs(floor(y1)-y) + 1; 
 
% initialize stepping criteria 
if (dx < 0.00001)
    x_inc = 0.0; 
    dt_dx = inf;
    t_next_horizontal = inf; 
    t_last_horizontal = inf; 
elseif (x1 > x0) 
    x_inc = 1.0; 
    dt_dx = 1.0 / dx; 
    t_next_horizontal = (x + 1.0 - x0) * dt_dx; % remember x is "floor(x0)" here 
    t_last_horizontal = (x0 - x) * dt_dx; 
else 
    x_inc = -1.0; 
    dt_dx = 1.0 / dx; 
    t_next_horizontal = (x0 - x) * dt_dx; % remember x is "floor(x0)" here 
    t_last_horizontal = (x + 1.0 - x0) * dt_dx; 
end
 
if (dy < 0.00001)
    y_inc = 0.0; 
    dt_dy = inf;
    t_next_vertical = inf; 
    t_last_vertical = inf;  
elseif (y1 > y0) 
    y_inc = 1.0; 
    dt_dy = 1.0 / dy; 
    t_next_vertical = (y + 1.0 - y0) * dt_dy; % remember y is "floor(y0)" here 
    t_last_vertical = (y0 - y) * dt_dy;  
else
    y_inc = -1.0; 
    dt_dy = 1.0 / dy; 
    t_next_vertical = (y0 - y) * dt_dy; % remember y is "floor(y0)" here 
    t_last_vertical = (y + 1.0 - y0) * dt_dy; 
end
 
% find cell intersection in opposite direction to initialize cell entrance 
% condition 
last_step_was_vert = (t_last_vertical < t_last_horizontal); 
 
% initialize entrance height 
h_entr = h0; 
 
for i = 0:n-1 

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
    % TODO: should be a way to get rid of this if statement by looking at dh outside for loop... 
    h_exit = h0 + dh * t; 
    if (dh > 0.0) 
        h_check = h_entr;  
    else
        h_check = h_exit; 
    end 
    h_entr = h_exit; 
    
    % bound corner coordinates 
    x_check = constrain(x, 0, LEN_IDX_E_1); 
    y_check = constrain(y, 0, LEN_IDX_N_1); 
    x_check1 = constrain(x_check+1, 0, LEN_IDX_E_1); 
    y_check1 = constrain(y_check+1, 0, LEN_IDX_N_1); 
    % convert to row-major indices 
    idx_corner1 = y_check*LEN_IDX_E + x_check +1; 
    idx_corner2 = y_check1*LEN_IDX_E + x_check +1; 
    idx_corner3 = y_check1*LEN_IDX_E + x_check1 +1; 
    idx_corner4 = y_check*LEN_IDX_E + x_check1 +1; 
    % check the four corners 
    check1 = terr_map(idx_corner1) > h_check; % corner 1 (bottom left) 
    check2 = terr_map(idx_corner2) > h_check; % corner 2 (top left) 
    check3 = terr_map(idx_corner3) > h_check; % corner 3 (top right) 
    check4 = terr_map(idx_corner4) > h_check; % corner 4 (bottom right)  

    % check cell triangles 
    if (last_step_was_vert)  % / / / / / / / / / / / / / / / / / / / / / 
    % vertical entrance step 

        if (take_vert_step) 
        % next step is vertical 

            if (y_inc > 0) %TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop... 
            % BR, TL 

                % check bottom-right triangle corners 
                if (check1 || check4 || check3)

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 
                end 

                % check top-left triangle corners 
                if ((check1 || check2 || check3) && (occ_detected==0))

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 
                end
            else 
            % TL, BR 

                % check top-left triangle corners 
                if (check1 || check2 || check3) 

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 
                end
                
                % check bottom-right triangle corners 
                if ((check1 || check4 || check3) && occ_detected==0) 

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 
                end
            end
        else  % - - - - - - - - - - - - - - - - - - - - - - - - - - -
        % next step is horizontal 

            if (y_inc > 0 && x_inc > 0)
            % BR 

                % check bottom-right triangle corners 
                if (check1 || check4 || check3)

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 
                end
                
            elseif (y_inc < 0 && x_inc < 0)
            % TL 

                % check top-left triangle corners 
                if (check1 || check2 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 
                end
            else  
            % BR, TL 

                % check bottom-right triangle corners 
                if (check1 || check4 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection          
                end
                
                % check top-left triangle corners 
                if ((check1 || check2 || check3) && (occ_detected==0))  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 
                end
            end
        end
    else  % last step was horizontal / / / / / / / / / / / / / / / / / /  
        
        if (take_vert_step)  
        % next step is vertical 

            if (x_inc > 0)  
            % TL 

                % check top-left triangle corners 
                if (check1 || check2 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 
     
                end
                            
                if ((y_inc < 0) && (occ_detected==0))  
                % BR 

                    % check bottom-right triangle corners 
                    if (check1 || check4 || check3)  

                        % set 3 corners 
                        p1(1) = terr_dis*x; 
                        p1(2) = terr_dis*y; 
                        p1(3) = terr_map(idx_corner1); 
                        p2(1) = terr_dis*(x+1); 
                        p2(2) = terr_dis*y; 
                        p2(3) = terr_map(idx_corner4); 
                        p3(1) = terr_dis*(x+1); 
                        p3(2) = terr_dis*(y+1); 
                        p3(3) = terr_map(idx_corner3); 

                        % check for ray-triangle intersection 
                        [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                        occ_detected = occ_detected + ret; % =1 if detection 

                    end
                end
            else
            % BR 

                % check bottom-right triangle corners 
                if (check1 || check4 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 
                end

                if ((y > 0) && (occ_detected==0))  
                % TL 

                    % check top-left triangle corners 
                    if (check1 || check2 || check3)  

                        % set 3 corners 
                        p1(1) = terr_dis*x; 
                        p1(2) = terr_dis*y; 
                        p1(3) = terr_map(idx_corner1); 
                        p2(1) = terr_dis*x; 
                        p2(2) = terr_dis*(y+1); 
                        p2(3) = terr_map(idx_corner2); 
                        p3(1) = terr_dis*(x+1); 
                        p3(2) = terr_dis*(y+1); 
                        p3(3) = terr_map(idx_corner3); 

                        % check for ray-triangle intersection 
                        [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                        occ_detected = occ_detected + ret*2; % =2 if detection 
                    end
                end
            end
        else  % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
        % next step is horizontal 

            if (x_inc > 0)  
            % TL, BR 

                % check top-left triangle corners 
                if (check1 || check2 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 

                end

                % check bottom-right triangle corners 
                if ((check1 || check4 || check3) && (occ_detected==0))  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 
                end
            
            else  
            % BR, TL 

                % check bottom-right triangle corners 
                if (check1 || check4 || check3)  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*(x+1); 
                    p2(2) = terr_dis*y; 
                    p2(3) = terr_map(idx_corner4); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret; % =1 if detection 

                end
                
                % check top-left triangle corners 
                if ((check1 || check2 || check3) && (occ_detected==0))  

                    % set 3 corners 
                    p1(1) = terr_dis*x; 
                    p1(2) = terr_dis*y; 
                    p1(3) = terr_map(idx_corner1); 
                    p2(1) = terr_dis*x; 
                    p2(2) = terr_dis*(y+1); 
                    p2(3) = terr_map(idx_corner2); 
                    p3(1) = terr_dis*(x+1); 
                    p3(2) = terr_dis*(y+1); 
                    p3(3) = terr_map(idx_corner3); 

                    % check for ray-triangle intersection 
                    [ret, d_occ, p_occ] = intersect_triangle(r0_rel, v, p1, p2, p3); 

                    occ_detected = occ_detected + ret*2; % =2 if detection 

                end
            end
        end
    end
    % / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / 

    % return if occlusion detected 
    if (occ_detected > 0)  
         return;
    end

    % actually take the step 
    if (take_vert_step)  % (t_next_vertical < t_next_horizontal) 
        % take a vertical step 
        y = y + y_inc; 
    else  
        % take a horizontal step 
        x = x + x_inc; 
    end
    last_step_was_vert = take_vert_step; 
end
 