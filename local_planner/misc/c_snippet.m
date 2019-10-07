castray(double *d_occ, double *p_occ, double *p1, double *p2, double *p3,
    const double r0[3], const double r1[3], const double v[3],
    const double pos_n_origin, const double pos_e_origin, const double terr_dis, double *terr_map) {

/* INPUTS:
 *
 * (double) r0[3]             	start position (e,n,u) [m]
 * (double) r1[3]             	end position (e,n,u) [m]
 * (double) v[3]              	ray unit vector (e,n,u)
 * (double) pos_n_origin        northing origin of terrain map
 * (double) pos_e_origin        easting origin of terrain map
 * (double) terr_dis          	terrain discretization
 * (double) terr_map          	terrain map
 *
 * OUTPUTS:
 *
 * (int)   occ_detected         0=no detection, 1=BR triangle detected, 2=TL triangle detected
 * (double) d_occ           	distance to the ray-triangle intersection [m]
 * (double) p_occ[3]        	coord. of the ray-triangle intersection [m]
 * (double) p1[3]           	coord. of triangle vertex 1 (e,n,u) [m]
 * (double) p2[3]             	coord. of triangle vertex 2 (e,n,u) [m]
 * (double)	p3[3]           	coord. of triangle vertex 3 (e,n,u) [m]
 */ 

// initialize
int occ_detected = 0;

// relative start position
const double x0 = (r0[0] - pos_e_origin)/terr_dis;
const double y0 = (r0[1] - pos_n_origin)/terr_dis;

// initial height
const double h0 = r0[2];

// relative end position
const double x1 = r1[0]/terr_dis;
const double y1 = r1[1]/terr_dis;

// end height
const double h1 = r1[2];

// line deltas
const double dx = fabs(x1 - x0);
const double dy = fabs(y1 - y0);

// initial cell origin
int x = int(floor(x0));
int y = int(floor(y0));

// unit change in line length per x/y (see definition below)
double dt_dx;
double dt_dy;

// change in height per unit line length (t)
const double dh = h1 - h0;

// number of cells we pass through
int n = int(floor(x1)-x + floor(y1)-y) + 1;

// initialize stepping criteria
double t_next_horizontal, t_last_horizontal;
double t_next_vertical, t_last_vertical;
double x_inc, y_inc;

if (dx < 0.00001) {
    x_inc = 0.0;
    t_next_horizontal = INFINITY;
    t_last_horizontal = INFINITY;
}
else if (x1 > x0) {
    x_inc = 1.0;
    dt_dx = 1.0 / dx;
    t_next_horizontal = (x + 1.0 - x0) * dt_dx; // remember x is "floor(x0)" here
    t_last_horizontal = (x0 - x) * dt_dx;
}
else {
    x_inc = -1.0;
    dt_dx = 1.0 / dx;
    t_next_horizontal = (x0 - x) * dt_dx; // remember x is "floor(x0)" here
    t_last_horizontal = (x + 1.0 - x0) * dt_dx;
}

if (dy < 0.00001) {
    y_inc = 0.0;
    t_next_vertical = INFINITY;
    t_last_vertical = INFINITY;
}
else if (y1 > y0) {
    y_inc = 1.0;
    dt_dy = 1.0 / dy;
    t_next_vertical = (y + 1.0 - y0) * dt_dy; // remember y is "floor(y0)" here
    t_last_vertical = (y0 - y) * dt_dy;
}
else {
    y_inc = -1.0;
    dt_dy = 1.0 / dy;
    t_next_vertical = (y0 - y) * dt_dy; // remember y is "floor(y0)" here
    t_last_vertical = (y + 1.0 - y0) * dt_dy;
}

// find cell intersection in opposite direction to initialize cell entrance
// condition
bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

// initialize entrance height
double h_entr = h0;

// for loop init
int x_check, y_check, x_check1, y_check1, idx_corner1, idx_corner2, idx_corner3, idx_corner4, ret;
double t, h_exit, h_check;
bool take_vert_step, check1, check2, check3, check4;

int i;
for (i = 0; i < n; i=i+1) {

    // bound corner coordinates
    x_check = constrain_int(x, 0, LEN_IDX_E_1);
    y_check = constrain_int(y, 0, LEN_IDX_N_1);
    x_check1 = constrain_int(x_check+1, 0, LEN_IDX_E_1);
    y_check1 = constrain_int(y_check+1, 0, LEN_IDX_N_1);
    // convert to row-major indices
    idx_corner1 = y_check*LEN_IDX_E + x_check;
    idx_corner2 = y_check1*LEN_IDX_E + x_check;
    idx_corner3 = y_check1*LEN_IDX_E + x_check1;
    idx_corner4 = y_check*LEN_IDX_E + x_check1;
    // check the four corners
    check1 = terr_map[idx_corner1] > h_check; // corner 1 (bottom left)
    check2 = terr_map[idx_corner2] > h_check; // corner 2 (top left)
    check3 = terr_map[idx_corner3] > h_check; // corner 3 (top right)
    check4 = terr_map[idx_corner4] > h_check; // corner 4 (bottom right) 

    // check the next step we will take and compute the exit height
    if (t_next_vertical < t_next_horizontal) {
        // next step is vertical
        take_vert_step = true;
        t = t_next_vertical; // current step
        t_next_vertical = t_next_vertical + dt_dy;
    }
    else {
        // next step is horizontal
        take_vert_step = false;
        t = t_next_horizontal; // current step
        t_next_horizontal = t_next_horizontal + dt_dx;
    }

    // take minimum of entrance and exit height for check
    // TODO: should be a way to get rid of this if statement by looking at dh outside for loop...
    h_exit = h0 + dh * t;
    if (dh > 0.0) {
        h_check = h_entr;
    }
    else {
        h_check = h_exit;
    }
    h_entr = h_exit;

    // check cell triangles
    if (last_step_was_vert) { // / / / / / / / / / / / / / / / / / / / / /
        // vertical entrance step

        if (take_vert_step) {
            // next step is vertical

            if (y_inc > 0) { //TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop...
                // BR, TL

                // check bottom-right triangle corners
                if (check1 || check4 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }

                // check top-left triangle corners
                if ((check1 || check2 || check3) && (occ_detected==0)) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }
            }
            else {
                // TL, BR

                // check top-left triangle corners
                if (check1 || check2 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }

                // check bottom-right triangle corners
                if ((check1 || check4 || check3) && occ_detected==0) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }
            }
        }   
        else  {// - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // next step is horizontal

            if (y_inc > 0 && x_inc > 0) {
                // BR

                // check bottom-right triangle corners
                if (check1 || check4 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }
            }
            else if (y_inc < 0 && x_inc < 0) {
                // TL

                // check top-left triangle corners
                if (check1 || check2 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }
            }
            else {
                // BR, TL

                // check bottom-right triangle corners
                if (check1 || check4 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }

                // check top-left triangle corners
                if ((check1 || check2 || check3) && (occ_detected==0)) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }
            }
        }
    }
    else { // last step was horizontal / / / / / / / / / / / / / / / / / / 
        if (take_vert_step) {
            // next step is vertical

            if (x_inc > 0) {
                // TL

                // check top-left triangle corners
                if (check1 || check2 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }

                if ((y_inc < 0) && (occ_detected==0)) {
                    // BR

                    // check bottom-right triangle corners
                    if (check1 || check4 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*(x+1);
                        p2[1] = terr_dis*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                        occ_detected += ret; // =1 if detection
                    }
                }
            }
            else {
                // BR

                // check bottom-right triangle corners
                if (check1 || check4 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }

                if ((y > 0) && (occ_detected==0)) {
                    // TL

                    // check top-left triangle corners
                    if (check1 || check2 || check3) {

                        // set 3 corners
                        p1[0] = terr_dis*x;
                        p1[1] = terr_dis*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = terr_dis*x;
                        p2[1] = terr_dis*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = terr_dis*(x+1);
                        p3[1] = terr_dis*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        // check for ray-triangle intersection
                        ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                        occ_detected += ret*2; // =2 if detection
                    }
                }
            }
        }
        else { // - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // next step is horizontal

            if (x_inc > 0) {
                // TL, BR

                // check top-left triangle corners
                if (check1 || check2 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }

                // check bottom-right triangle corners
                if ((check1 || check4 || check3) && (occ_detected==0)) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }
            }
            else {
                // BR, TL

                // check bottom-right triangle corners
                if (check1 || check4 || check3) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*(x+1);
                    p2[1] = terr_dis*y;
                    p2[2] = terr_map[idx_corner4];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret; // =1 if detection
                }

                // check top-left triangle corners
                if ((check1 || check2 || check3) && (occ_detected==0)) {

                    // set 3 corners
                    p1[0] = terr_dis*x;
                    p1[1] = terr_dis*y;
                    p1[2] = terr_map[idx_corner1];
                    p2[0] = terr_dis*x;
                    p2[1] = terr_dis*(y+1);
                    p2[2] = terr_map[idx_corner2];
                    p3[0] = terr_dis*(x+1);
                    p3[1] = terr_dis*(y+1);
                    p3[2] = terr_map[idx_corner3];

                    // check for ray-triangle intersection
                    ret = intersect_triangle(d_occ, p_occ, r0, v, p1, p2, p3);

                    occ_detected += ret*2; // =2 if detection
                }
            }
        }
    } // / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

    // return if occlusion detected
    if (occ_detected > 0) {
        return occ_detected;
    }

    // actually take the step
    if (take_vert_step) { // (t_next_vertical < t_next_horizontal)
        // take a vertical step
        y = y + y_inc;
    }
    else {
        // take a horizontal step
        x = x + x_inc;
    }
    last_step_was_vert = take_vert_step;
}
return occ_detected;
}