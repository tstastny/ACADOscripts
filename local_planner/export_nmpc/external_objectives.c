/* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */
/* EXTERNAL FUNCTIONS / / / / / / / / / / / / / / / / / / / / / / / / / /*/
/*
 * below are a collection of functions to be called from outside the MPC
 * internal model.
 */

/* TERRAIN CHECKING / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

void lookup_terrain_idx(const double pos_n, const double pos_e, const double pos_n_origin,
        const double pos_e_origin, const int map_height, const int map_width,
        const double map_resolution, int *idx_q, double *dn, double *de)
{
    
    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;
            
    /* relative position / indices */
    const double rel_n = pos_n - pos_n_origin;
    const double rel_n_bar = rel_n / map_resolution;
    int idx_n = (int)(floor(rel_n_bar));
    const double rel_e = pos_e - pos_e_origin;
    const double rel_e_bar = rel_e / map_resolution;
    int idx_e = (int)(floor(rel_e_bar));
    
    /* interpolation weights */
    *dn = rel_n_bar-idx_n;
    *de = rel_e_bar-idx_e;
    
    /* cap ends */
    if (idx_n < 0) {
        idx_n = 0;
    }
    else if (idx_n > map_height_1) {
        idx_n = map_height_1;
    }
    if (idx_e < 0) {
        idx_e = 0;
    }
    else if (idx_e > map_width_1) {
        idx_e = map_width_1;
    }
    
    /* neighbors (north) */
    int q_n[4];
    if (idx_n >= map_height_1) {
        q_n[0] = map_height_1;
        q_n[1] = map_height_1;
        q_n[2] = map_height_1;
        q_n[3] = map_height_1;
    }
    else {
        q_n[0] = idx_n;
        q_n[1] = idx_n + 1;
        q_n[2] = idx_n;
        q_n[3] = idx_n + 1;
    }
    /* neighbors (east) */
    int q_e[4];
    if (idx_e >= map_width_1) {
        q_e[0] = map_width_1;
        q_e[1] = map_width_1;
        q_e[2] = map_width_1;
        q_e[3] = map_width_1;
    }
    else {
        q_e[0] = idx_e;
        q_e[1] = idx_e;
        q_e[2] = idx_e + 1;
        q_e[3] = idx_e + 1;
    }
    
    /* neighbors row-major indices */
    idx_q[0] = q_n[0]*map_width + q_e[0];
    idx_q[1] = q_n[1]*map_width + q_e[1];
    idx_q[2] = q_n[2]*map_width + q_e[2];
    idx_q[3] = q_n[3]*map_width + q_e[3];
}

/* check ray-triangle intersection */
int intersect_triangle(double *d_occ, double *p_occ, double *n_occ,
        const double r0[3], const double v_ray[3],
        const double p1[3], const double p2[3], const double p3[3], const int v_dir) {
    /* following: "Fast, Minimum Storage Ray/Triangle Intersection",
     * Moeller et. al., Journal of Graphics Tools, Vol.2(1), 1997
     */

    /* NOTE: all vectors in here are E,N,U */
    
    /* find vectors for two edges sharing p1 */
    const double e1[3] = {p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]};
    const double e2[3] = {p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]};

    /* begin calculating determinant - also used to calculate U parameter */
    double pvec[3];
    cross(pvec, v_ray, e2);

    /* we don't test for backwards culling here as, assuming this ray casting
     * algorithm is properly detecting the first occluding triangles, we should
     * not run into a case of a true "backwards" facing triangle (also the
     * current BR and TL definitions have opposite vertex spin definitions..
     * should probably change that in the future.. could maybe avoid a few
     * divisions of the determinant) */

    /* if the determinant is near zero, ray lies in the triangle plane */
    const double det = dot(e1, pvec);
    if (det > -EPSILON && det < EPSILON) {
        return 0;
    }
    
    /* divide the determinant (XXX: could possibly find a way to avoid this until the last minute possible..) */
    const double inv_det = 1.0 / det;

    /* calculate distance from p1 to ray origin */
    const double tvec[3] = {r0[0] - p1[0], r0[1] - p1[1], r0[2] - p1[2]};

    /* calculate u parameter and test bounds */
    const double u = dot(tvec, pvec) * inv_det;
    if (u < 0.0 || u > 1.0) {
        return 0;
    }
    
    /* prepare to test v parameter */
    double qvec[3];
    cross(qvec, tvec, e1);

    /* calculate v parameter and test bounds */
    const double v = dot(v_ray, qvec) * inv_det;
    if (v < 0.0 || u + v > 1.0) {
        return 0;
    }

    /* calculate d_occ, scale parameters, ray intersects triangle */
    *d_occ = dot(e2, qvec) * inv_det;

    /* calculate and return intersection point */
    const double one_u_v = (1 - u - v);
    p_occ[0] = one_u_v * p1[0] + u * p2[0] + v * p3[0];
    p_occ[1] = one_u_v * p1[1] + u * p2[1] + v * p3[1];
    p_occ[2] = one_u_v * p1[2] + u * p2[2] + v * p3[2];
    
    /* calculate and return plane normal */
    cross(n_occ, e2, e1);
    const double one_over_norm_n_occ = 1.0/sqrt(dot(n_occ,n_occ));
    n_occ[0] *= v_dir * one_over_norm_n_occ;
    n_occ[1] *= v_dir * one_over_norm_n_occ;
    n_occ[2] *= v_dir * one_over_norm_n_occ;
    
    return 1;
}

/* cast ray through terrain map and determine the intersection point on any occluding trianglular surface */
int castray(double *r_occ, double *p_occ, double *n_occ, double *p1, double *p2, double *p3,
        const double r0[3], const double r1[3], const double v[3],
        const double pos_n_origin, const double pos_e_origin, const int map_height,
        const int map_width, const double map_resolution, const double *terr_map) {
    
    /* INPUTS:
     *
     * (double) r0[3]             	start position (e,n,u) [m]
     * (double) r1[3]             	end position (e,n,u) [m]
     * (double) v[3]              	ray unit vector (e,n,u)
     * (double) pos_n_origin        northing origin of terrain map
     * (double) pos_e_origin        easting origin of terrain map
     * (double) map_resolution          	terrain discretization
     * (double) terr_map          	terrain map
     *
     * OUTPUTS:
     *
     * (int)   occ_detected         0=no detection, 1=BR triangle detected, 2=TL triangle detected
     * (double) d_occ           	distance to the ray-triangle intersection [m]
     * (double) p_occ[3]        	coord. of the ray-triangle intersection [m]
     * (double) n_occ[3]            plane unit normal vector
     * (double) p1[3]           	coord. of triangle vertex 1 (e,n,u) [m]
     * (double) p2[3]             	coord. of triangle vertex 2 (e,n,u) [m]
     * (double)	p3[3]           	coord. of triangle vertex 3 (e,n,u) [m]
     */ 
    
    const int map_height_1 = map_height - 1;
    const int map_width_1 = map_width - 1;

    /* initialize */
    int occ_detected = 0;

    /* relative (unit) start position */
    const double x0 = (r0[0] - pos_e_origin)/map_resolution;
    const double y0 = (r0[1] - pos_n_origin)/map_resolution;

    /* initial height */
    const double h0 = r0[2];
    
    /* vector for triangle intersect inputs */
    const double r0_rel[3] = {x0*map_resolution,y0*map_resolution,h0}; /*XXX: this origin subtracting/adding is inefficient.. pick one and go with it for this function

    /* relative end position */
    const double x1 = (r1[0] - pos_e_origin)/map_resolution;
    const double y1 = (r1[1] - pos_n_origin)/map_resolution;

    /* end height */
    const double h1 = r1[2];

    /* line deltas */
    const double dx = fabs(x1 - x0);
    const double dy = fabs(y1 - y0);

    /* initial cell origin */
    int x = (int)(floor(x0));
    int y = (int)(floor(y0));

    /* unit change in line length per x/y (see definition below) */
    double dt_dx;
    double dt_dy;

    /* change in height per unit line length (t) */
    const double dh = h1 - h0;

    /* number of cells we pass through */
    int n = fabs(floor(x1)-x) + fabs(floor(y1)-y) + 1; /*XXX: what is the real difference between this and using dx / dy? */

    /* initialize stepping criteria */
    double t_next_horizontal, t_last_horizontal;
    double t_next_vertical, t_last_vertical;
    double x_inc, y_inc;

    if (dx < 0.00001) {
        x_inc = 0.0;
        dt_dx = INFINITY;
        t_next_horizontal = INFINITY;
        t_last_horizontal = INFINITY;
    }
    else if (x1 > x0) {
        x_inc = 1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x + 1.0 - x0) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x0 - x) * dt_dx;
    }
    else {
        x_inc = -1.0;
        dt_dx = 1.0 / dx;
        t_next_horizontal = (x0 - x) * dt_dx; /* remember x is "floor(x0)" here */
        t_last_horizontal = (x + 1.0 - x0) * dt_dx;
    }

    if (dy < 0.00001) {
        y_inc = 0.0;
        dt_dy = INFINITY;
        t_next_vertical = INFINITY;
        t_last_vertical = INFINITY;
    }
    else if (y1 > y0) {
        y_inc = 1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y + 1.0 - y0) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y0 - y) * dt_dy;
    }
    else {
        y_inc = -1.0;
        dt_dy = 1.0 / dy;
        t_next_vertical = (y0 - y) * dt_dy; /* remember y is "floor(y0)" here */
        t_last_vertical = (y + 1.0 - y0) * dt_dy;
    }

    /* find cell intersection in opposite direction to initialize cell entrance
     * condition */
    bool last_step_was_vert = (t_last_vertical < t_last_horizontal);

    /* initialize entrance height */
    double h_entr = h0;

    /* for loop init */
    int ret;
    double t, h_exit, h_check;
    bool take_vert_step, check1, check2, check3, check4;
    
    /* check that start position is not already under the terrain */
    
    /* bound corner coordinates */
    int x_check = constrain_int(x, 0, map_width_1);
    int y_check = constrain_int(y, 0, map_height_1);
    int x_check1 = constrain_int(x_check+1, 0, map_width_1);
    int y_check1 = constrain_int(y_check+1, 0, map_height_1);
    /* convert to row-major indices */
    int idx_corner1 = y_check*map_width + x_check;
    int idx_corner2 = y_check1*map_width + x_check;
    int idx_corner3 = y_check1*map_width + x_check1;
    int idx_corner4 = y_check*map_width + x_check1;
    
    const double x0_unit = x0 - x;
    const double y0_unit = y0 - y;
    if (y0_unit > x0_unit) {
        /* check bottom-right triangle */
        if (x0_unit*(terr_map[idx_corner4]-terr_map[idx_corner1]) + y0_unit*(terr_map[idx_corner3] - terr_map[idx_corner4]) > h0) {
            return occ_detected;
        }
    }
    else {
        /* check top-left triangle */
        if (x0_unit*(terr_map[idx_corner3]-terr_map[idx_corner2]) + y0_unit*(terr_map[idx_corner2] - terr_map[idx_corner1]) > h0) {
            return occ_detected;
        }
    }

    /* cast the ray */
    int i;
    for (i = 0; i < n; i=i+1) {

        /* check the next step we will take and compute the exit height */
        if (t_next_vertical < t_next_horizontal) {
            /* next step is vertical */
            take_vert_step = true;
            t = t_next_vertical; /* current step */
            t_next_vertical = t_next_vertical + dt_dy;
        }
        else {
            /* next step is horizontal */
            take_vert_step = false;
            t = t_next_horizontal; /* current step */
            t_next_horizontal = t_next_horizontal + dt_dx;
        }

        /* take minimum of entrance and exit height for check */
        /* TODO: should be a way to get rid of this if statement by looking at dh outside for loop... */
        h_exit = h0 + dh * t;
        if (dh > 0.0) {
            h_check = h_entr;
        }
        else {
            h_check = h_exit;
        }
        h_entr = h_exit;
        
        /* bound corner coordinates */
        x_check = constrain_int(x, 0, map_width_1);
        y_check = constrain_int(y, 0, map_height_1);
        x_check1 = constrain_int(x_check+1, 0, map_width_1);
        y_check1 = constrain_int(y_check+1, 0, map_height_1);
        /* convert to row-major indices */
        idx_corner1 = y_check*map_width + x_check;
        idx_corner2 = y_check1*map_width + x_check;
        idx_corner3 = y_check1*map_width + x_check1;
        idx_corner4 = y_check*map_width + x_check1;
        /* check the four corners */
        check1 = terr_map[idx_corner1] > h_check; /* corner 1 (bottom left) */
        check2 = terr_map[idx_corner2] > h_check; /* corner 2 (top left) */
        check3 = terr_map[idx_corner3] > h_check; /* corner 3 (top right) */
        check4 = terr_map[idx_corner4] > h_check; /* corner 4 (bottom right) */

        /* check cell triangles */
        if (last_step_was_vert) { /* / / / / / / / / / / / / / / / / / / */
            /* vertical entrance step */

            if (take_vert_step) {
                /* next step is vertical */

                if (y_inc > 0) { /*TODO: should be able to get rid of a few of these ifs by making the decision outside the for loop... */
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && occ_detected==0) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
            }   
            else  {/* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (y_inc > 0 && x_inc > 0) {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else if (y_inc < 0 && x_inc < 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        }
        else { /* last step was horizontal / / / / / / / / / / / / / / / */
            if (take_vert_step) {
                /* next step is vertical */

                if (x_inc > 0) {
                    /* TL */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    if ((y_inc < 0) && (occ_detected==0)) {
                        /* BR */

                        /* check bottom-right triangle corners */
                        if (check1 || check4 || check3) {

                            /* set 3 corners */
                            p1[0] = map_resolution*x;
                            p1[1] = map_resolution*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = map_resolution*(x+1);
                            p2[1] = map_resolution*y;
                            p2[2] = terr_map[idx_corner4];
                            p3[0] = map_resolution*(x+1);
                            p3[1] = map_resolution*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                            occ_detected += ret; /* =1 if detection */
                        }
                    }
                }
                else {
                    /* BR */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    if ((y > 0) && (occ_detected==0)) {
                        /* TL */

                        /* check top-left triangle corners */
                        if (check1 || check2 || check3) {

                            /* set 3 corners */
                            p1[0] = map_resolution*x;
                            p1[1] = map_resolution*y;
                            p1[2] = terr_map[idx_corner1];
                            p2[0] = map_resolution*x;
                            p2[1] = map_resolution*(y+1);
                            p2[2] = terr_map[idx_corner2];
                            p3[0] = map_resolution*(x+1);
                            p3[1] = map_resolution*(y+1);
                            p3[2] = terr_map[idx_corner3];

                            /* check for ray-triangle intersection */
                            ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                            occ_detected += ret*2; /* =2 if detection */
                        }
                    }
                }
            }
            else { /* - - - - - - - - - - - - - - - - - - - - - - - - - -*/
                /* next step is horizontal */

                if (x_inc > 0) {
                    /* TL, BR */

                    /* check top-left triangle corners */
                    if (check1 || check2 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }

                    /* check bottom-right triangle corners */
                    if ((check1 || check4 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }
                }
                else {
                    /* BR, TL */

                    /* check bottom-right triangle corners */
                    if (check1 || check4 || check3) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*(x+1);
                        p2[1] = map_resolution*y;
                        p2[2] = terr_map[idx_corner4];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, -1);

                        occ_detected += ret; /* =1 if detection */
                    }

                    /* check top-left triangle corners */
                    if ((check1 || check2 || check3) && (occ_detected==0)) {

                        /* set 3 corners */
                        p1[0] = map_resolution*x;
                        p1[1] = map_resolution*y;
                        p1[2] = terr_map[idx_corner1];
                        p2[0] = map_resolution*x;
                        p2[1] = map_resolution*(y+1);
                        p2[2] = terr_map[idx_corner2];
                        p3[0] = map_resolution*(x+1);
                        p3[1] = map_resolution*(y+1);
                        p3[2] = terr_map[idx_corner3];

                        /* check for ray-triangle intersection */
                        ret = intersect_triangle(r_occ, p_occ, n_occ, r0_rel, v, p1, p2, p3, 1);

                        occ_detected += ret*2; /* =2 if detection */
                    }
                }
            }
        } /* / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / */

        /* return if occlusion detected */
        if (occ_detected > 0) {
            return occ_detected;
        }

        /* actually take the step */
        if (take_vert_step) { /* (t_next_vertical < t_next_horizontal) */
            /* take a vertical step */
            y = y + y_inc;
        }
        else {
            /* take a horizontal step */
            x = x + x_inc;
        }
        last_step_was_vert = take_vert_step;
    }
    return occ_detected;
}

/* GUIDANCE / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* calculate ground velocity reference */
void calculate_velocity_reference(double *v_ref, double *e_lat, double *e_lon,
        const double *states,
        const double *path_reference,
        const double *guidance_params,
        const double *speed_states,
        const double *jac_sig_r,
        const double prio_r,
        const int path_type)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    
    /* path reference */
    const double b_n = path_reference[0];
    const double b_e = path_reference[1];
    const double b_d = path_reference[2];
    
    /* guidance */
    const double T_b_lat = guidance_params[0];
    const double T_b_lon = guidance_params[1];
    const double gamma_app_max = guidance_params[2];
    bool use_occ_as_guidance = (guidance_params[3] > 0.5);
    
    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    
    /* path following - - - - - - - - - - - - - - - - - - - - - - - - - -*/
    
    double vP_n_unit = 0.0;
    double vP_e_unit = 0.0;
    double vP_d_unit = 0.0;
    
    if (path_type == 0) {
        /* loiter at fixed altitude */
        
        const double Gamma_p = 0.0;
        
        /* loiter direction (hijack chi_p param) */
        const double loiter_dir = (path_reference[4] < 0.0) ? -1.0 : 1.0;
        const double radius = (fabs(path_reference[4]) < 0.1) ? 0.1 : fabs(path_reference[4]);
        
        /* vector from circle center to aircraft */
        const double br_n = r_n-b_n;
        const double br_e = r_e-b_e;
        const double br_d = r_d-b_d;
        
        /* lateral-directional distance to circle center */
        const double dist_to_center = sqrt(br_n*br_n + br_e*br_e);
        
        /* norm of lateral-directional ground velocity */
        const double vG_lat = sqrt(vG_n*vG_n + vG_e*vG_e);

        double br_n_unit, br_e_unit;
        double p_n, p_e;
        if (dist_to_center < 0.1) {
            if (vG_lat < 0.1) {
                /* arbitrarily set the point in the northern top of the circle */
                br_n_unit = 1.0;
                br_e_unit = 0.0;
                
                /* closest point on circle */
                p_n = b_n + br_n_unit * radius;
                p_e = b_e + br_e_unit * radius;
            }
            else {
                /* set the point in the direction we are moving */
                br_n_unit = vG_n / vG_lat * 0.1;
                br_e_unit = vG_e / vG_lat * 0.1;
                
                /* closest point on circle */
                p_n = b_n + br_n_unit * radius;
                p_e = b_e + br_e_unit * radius;
            }
        }
        else {
            /* set the point in the direction of the aircraft */
            br_n_unit = br_n / dist_to_center;
            br_e_unit = br_e / dist_to_center;
            
            /* closest point on circle */
            p_n = b_n + br_n_unit * radius;
            p_e = b_e + br_e_unit * radius;
        }
        
        /* path tangent unit vector */
        const double tP_n_bar = -br_e_unit * loiter_dir;
        const double tP_e_bar = br_n_unit * loiter_dir;
        const double chi_p = atan2(tP_e_bar, tP_n_bar);

        /* position error */
        *e_lat = (r_n-p_n)*tP_e_bar - (r_e-p_e)*tP_n_bar;
        *e_lon = b_d - r_d;
        
        /* lateral-directional error boundary */
        const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);

        /* course approach angle */
        const double chi_app = atan(M_PI_2*(*e_lat)/e_b_lat);

        /* longitudinal error boundary */
        double e_b_lon;
        if (fabs(vG_d) < 1.0) {
            e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); /* vG_d may be zero */
        }
        else {
            e_b_lon = T_b_lon * fabs(vG_d);
        }

        /* flight path approach angle */
        const double Gamma_app = -gamma_app_max * atan(M_PI_2*(*e_lon)/e_b_lon);

        /* normalized ground velocity setpoint */
        const double cos_gamma = cos(Gamma_p + Gamma_app);
        vP_n_unit = cos_gamma*cos(chi_p + chi_app);
        vP_e_unit = cos_gamma*sin(chi_p + chi_app);
        vP_d_unit = -sin(Gamma_p + Gamma_app);
    }
    else if (path_type == 1) {
        /* line */
        
        /* path direction */
        const double Gamma_p = path_reference[3];
        const double chi_p = path_reference[4];
        
        /* path tangent unit vector  */
        const double tP_n_bar = cos(chi_p);
        const double tP_e_bar = sin(chi_p);

        /* "closest" point on track */
        const double tp_dot_br = tP_n_bar*(r_n-b_n) + tP_e_bar*(r_e-b_e);
        const double tp_dot_br_n = tp_dot_br*tP_n_bar;
        const double tp_dot_br_e = tp_dot_br*tP_e_bar;
        const double p_lat = tp_dot_br_n*tP_n_bar + tp_dot_br_e*tP_e_bar;
        const double p_d = b_d - p_lat*tan(Gamma_p);

        /* position error */
        *e_lat = (r_n-b_n)*tP_e_bar - (r_e-b_e)*tP_n_bar;
        *e_lon = p_d - r_d;
        
        /* lateral-directional error boundary */
        const double e_b_lat = T_b_lat * sqrt(vG_n*vG_n + vG_e*vG_e);

        /* course approach angle */
        const double chi_app = atan(M_PI_2*(*e_lat)/e_b_lat);

        /* longitudinal error boundary */
        double e_b_lon;
        if (fabs(vG_d) < 1.0) {
            e_b_lon = T_b_lon * 0.5 * (1.0 + vG_d*vG_d); /* vG_d may be zero */
        }
        else {
            e_b_lon = T_b_lon * fabs(vG_d);
        }

        /* flight path approach angle */
        const double Gamma_app = -gamma_app_max * atan(M_PI_2*(*e_lon)/e_b_lon);

        /* normalized ground velocity setpoint */
        const double cos_gamma = cos(Gamma_p + Gamma_app);
        vP_n_unit = cos_gamma*cos(chi_p + chi_app);
        vP_e_unit = cos_gamma*sin(chi_p + chi_app);
        vP_d_unit = -sin(Gamma_p + Gamma_app);
    }
    else {
        /* unknown */
        /* fly north.. */
        
        /* position error */
        *e_lat = 0.0;
        *e_lon = 0.0;

        /* normalized ground velocity setpoint */
        vP_n_unit = 1.0;
        vP_e_unit = 0.0;
        vP_d_unit = 0.0;
    }
    
    if (use_occ_as_guidance) {
        /* terrain avoidance velocity setpoint */
        const double norm_jac_sig_r = sqrt(jac_sig_r[0]*jac_sig_r[0] + jac_sig_r[1]*jac_sig_r[1] + jac_sig_r[2]*jac_sig_r[2]);
        const double one_over_norm_jac_sig_r = (norm_jac_sig_r > 0.0001) ? 1.0/norm_jac_sig_r : 10000.0;
        const double v_occ_n_unit = -jac_sig_r[0] * one_over_norm_jac_sig_r;
        const double v_occ_e_unit = -jac_sig_r[1] * one_over_norm_jac_sig_r;
        const double v_occ_d_unit = -jac_sig_r[2] * one_over_norm_jac_sig_r;

        /* velocity errors */
        v_ref[0] = vP_n_unit * prio_r + (1.0-prio_r) * v_occ_n_unit;
        v_ref[1] = vP_e_unit * prio_r + (1.0-prio_r) * v_occ_e_unit;
        v_ref[2] = vP_d_unit * prio_r + (1.0-prio_r) * v_occ_d_unit;
    }
    else {
        /* velocity errors */
        v_ref[0] = vP_n_unit;
        v_ref[1] = vP_e_unit;
        v_ref[2] = vP_d_unit;
    }
}

/* SOFT CONSTRAINTS / / / / / / / / / / / / / / / / / / / / / / / / / / /*/

/* calculate soft angle of attack objective */
void calculate_aoa_objective(double *sig_aoa, double *jac_sig_aoa, double *prio_aoa,
        const double *states, const double *aoa_params)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    /*const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];*/
    const double gamma = states[4];
    /*const double xi = states[5];
    const double phi = states[6];*/
    const double theta = states[7];
    /*const double n_p = states[8];*/
    
    /* angle of attack soft constraint */
    const double delta_aoa = aoa_params[0];
    const double aoa_m = aoa_params[1];
    const double aoa_p = aoa_params[2];
    const double log_sqrt_w_over_sig1_aoa = aoa_params[3];
    const double one_over_sqrt_w_aoa = aoa_params[4];
    
    /* angle of attack */
    const double aoa = theta - gamma;
    *sig_aoa = 0.0;
    *prio_aoa = 1.0;
    jac_sig_aoa[0] = 0.0;
    jac_sig_aoa[1] = 0.0;

    /* angle of attack objective / jacobian - - - - - - - - - - - - - - -*/

    if (!(one_over_sqrt_w_aoa<0.0)) {

        /* upper bound */
        const double sig_aoa_p = (aoa - aoa_p < 0.0)
            ? exp((aoa - aoa_p)/delta_aoa*log_sqrt_w_over_sig1_aoa)
            : 1.0 + log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_p);

        /* lower bound */
        const double sig_aoa_m = (aoa - aoa_m > 0.0)
            ? exp(-(aoa - aoa_m)/delta_aoa*log_sqrt_w_over_sig1_aoa)
            : 1.0 - log_sqrt_w_over_sig1_aoa/delta_aoa * (aoa - aoa_m);

        /* combined */
        *sig_aoa = sig_aoa_p + sig_aoa_m;
        
        /* jacobian */
        if (aoa - aoa_p > 0.0) {
            /* upper linear jacobian */
            jac_sig_aoa[0] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else if (aoa - aoa_m < 0.0) {
            /* lower linear jacobian */
            jac_sig_aoa[0] = log_sqrt_w_over_sig1_aoa/delta_aoa; /* gamma */
            jac_sig_aoa[1] = -log_sqrt_w_over_sig1_aoa/delta_aoa; /* theta */
        }
        else {
            /* exponential jacobian */
            jacobian_sig_aoa_exp(jac_sig_aoa, delta_aoa, log_sqrt_w_over_sig1_aoa,
                sig_aoa_m, sig_aoa_p);
        }

        /* prioritization */
        *prio_aoa = 1.0; /* TODO: consider putting this back */
    }
}

/* calculate soft height objective */
void calculate_height_objective(double *sig_h, double *jac_sig_h, double *prio_h, double *h_terr,
        const double *states, const double *terr_params, const double terr_local_origin_n,
        const double terr_local_origin_e, const int map_height, const int map_width,
        const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    /*const double v = states[3];
    const double gamma = states[4];*/
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/
    
    /* height params */
    const double h_offset = terr_params[0];
    const double delta_h = terr_params[1];
    const double delta_y = 0.0;
    const double log_sqrt_w_over_sig1_h = terr_params[2];
    const double one_over_sqrt_w_h = terr_params[3];
    
    /* INTERMEDIATE CALCULATIONS - - - - - - - - - - - - - - - - - - - - */
    
    const double sin_xi = sin(xi);
    const double cos_xi = cos(xi);
    
    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* init */
    const double h = -r_d;
    *sig_h = 0.0;
    double sig_h_temp = 0.0;
    *prio_h = 1.0;
    jac_sig_h[0] = 0.0;
    jac_sig_h[1] = 0.0;
    jac_sig_h[2] = 0.0;
    jac_sig_h[3] = 0.0;
    double jac_sig_h_temp[4] = {0.0, 0.0, 0.0, 0.0};
    
    /* if not disabled by weight */
    if (!(one_over_sqrt_w_h<0.0)) {
        
        /* lookup 2.5d grid (CENTER) - - - - - - - - - - - - - - - - - - */
        int idx_q[4];
        double dn, de;    
        double sgn_n = 0.0;
        double sgn_e = 0.0;
        lookup_terrain_idx(r_n, r_e, terr_local_origin_n, terr_local_origin_e, map_height, map_width, map_resolution, idx_q, &dn, &de);
        
        /* bi-linear interpolation */
        double h12 = (1-dn)*terr_map[idx_q[0]] + dn*terr_map[idx_q[1]];
        double h34 = (1-dn)*terr_map[idx_q[2]] + dn*terr_map[idx_q[3]];
        double h_terr_temp = (1-de)*h12 + de*h34;
        *h_terr = h_terr_temp;
        
        /* objective / jacobian */
        const double sig_input = (h - h_terr_temp - h_offset)/delta_h;
        if (sig_input < 0.0) {
            /* linear */
            *sig_h = 1.0 + -log_sqrt_w_over_sig1_h * sig_input;
            
            jacobian_sig_h_lin(jac_sig_h,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e,
                sgn_n, map_resolution, xi);
        }
        else {
            /* exponential */
            *sig_h = exp(-sig_input*log_sqrt_w_over_sig1_h);
            
            jacobian_sig_h_exp(jac_sig_h,
                de, delta_h, delta_y,
                terr_map[idx_q[0]], h12, terr_map[idx_q[1]],
                terr_map[idx_q[2]], h34, terr_map[idx_q[3]],
                log_sqrt_w_over_sig1_h, sgn_e, sgn_n, *sig_h,
                map_resolution, xi);
        }

        /* prioritization */
        *prio_h = constrain_double(sig_input, 0.0, 1.0);
    }
}

/* calculate soft radial objective */
void calculate_radial_objective(double *sig_r, double *jac_sig_r, double *r_occ,
        double *p_occ, double *n_occ, double *prio_r, int *occ_detected,
        const double *v_ray, const double *states, const double *speed_states,
        const double *terr_params, const double terr_local_origin_n, const double terr_local_origin_e,
        const int map_height, const int map_width, const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    /*const double phi = states[6];
    const double theta = states[7];
    const double n_p = states[8];*/
    
    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    const double vG_sq = speed_states[6];
    const double vG_norm = speed_states[7];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];
    
    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];
    
    
    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* init */
    *sig_r = 0.0;
    *prio_r = 1.0;
    jac_sig_r[0] = 0.0;
    jac_sig_r[1] = 0.0;
    jac_sig_r[2] = 0.0;
    jac_sig_r[3] = 0.0;
    jac_sig_r[4] = 0.0;
    jac_sig_r[5] = 0.0;

    /* cast ray along ground speed vector to check for occlusions */
    
    /* init */
    double p1[3];
    double p2[3];
    double p3[3];
    
    /* relative velocity */
    const double vG_vec[3] = {vG_e, vG_n, -vG_d};
    double v_rel = dot(v_ray, vG_vec); /* in ENU */
    if (v_rel < 0.0) v_rel = 0.0;
    const double v_rel_sq = v_rel*v_rel;
    
    /* radial buffer zone */
    const double delta_r = delta_r0 + v_rel_sq * k_delta_r;
    
    /* adjusted radial offset */
    const double r_offset_1 = r_offset + v_rel_sq * k_r_offset;
    
    /* ray length */
    const double d_ray = delta_r + r_offset_1 + map_resolution;
    
    /* ray start ENU */
    const double r0[3] = {r_e, r_n, -r_d};
    /* ray end ENU */
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};
    
    /* cast the ray */
    *occ_detected = castray(r_occ, p_occ, n_occ, p1, p2, p3, r0, r1, v_ray,
            terr_local_origin_n, terr_local_origin_e, map_height, map_width,
            map_resolution, terr_map);
    
    /* shift occlusion origin */
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
    
    if (!(one_over_sqrt_w_r<0.0) && (*occ_detected>0)) {
        
        const double r_unit = (*r_occ - r_offset_1)/delta_r;
        
        double jac_r_unit[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
        
        jacobian_r_unit(jac_r_unit, 
            delta_r, gamma, k_delta_r, k_r_offset,
            n_occ[0], n_occ[2], n_occ[1],
            r_unit, v,
            v_ray[0], v_ray[2], v_ray[1],
            v_rel, xi);
        
        /* objective / jacobian */
        if (r_unit < 0.0) {
            /* linear */
            *sig_r = 1.0 - log_sqrt_w_over_sig1_r * r_unit;
            jac_sig_r[0] = -log_sqrt_w_over_sig1_r * jac_r_unit[0];
            jac_sig_r[1] = -log_sqrt_w_over_sig1_r * jac_r_unit[1];
            jac_sig_r[2] = -log_sqrt_w_over_sig1_r * jac_r_unit[2];
            jac_sig_r[3] = -log_sqrt_w_over_sig1_r * jac_r_unit[3];
            jac_sig_r[4] = -log_sqrt_w_over_sig1_r * jac_r_unit[4];
            jac_sig_r[5] = -log_sqrt_w_over_sig1_r * jac_r_unit[5];
        }
        else {
            /* exponential */
            *sig_r = exp(-r_unit*log_sqrt_w_over_sig1_r);
            const double mult_ = -log_sqrt_w_over_sig1_r * r_unit;
            jac_sig_r[0] = mult_ * jac_r_unit[0];
            jac_sig_r[1] = mult_ * jac_r_unit[1];
            jac_sig_r[2] = mult_ * jac_r_unit[2];
            jac_sig_r[3] = mult_ * jac_r_unit[3];
            jac_sig_r[4] = mult_ * jac_r_unit[4];
            jac_sig_r[5] = mult_ * jac_r_unit[5];
        }
        jac_sig_r[3] = 0.0; /* discourage mpc from using airspeed to combat costs */
        
        /* prioritization */
        *prio_r = constrain_double(r_unit, 0.0, 1.0);
    }
}

/* calculate unit radial distance and gradient */
void add_unit_radial_distance_and_gradient(double *jac_r_unit, double *r_unit_min, bool *f_min, int *occ_count,
        double *p_occ, double *n_occ, const double *states, const double *speed_states, const double *terr_params)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    
    /* speed states */
    const double vG_n = speed_states[3];
    const double vG_e = speed_states[4];
    const double vG_d = speed_states[5];
    
    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];
    
    /* CALCULATE OBJECTIVE - - - - - - - - - - - - - - - - - - - - - - - */
    
    double r_occ_vec[3] = {r_e - p_occ[0], r_n - p_occ[1], -r_d - p_occ[2]};
    
    /* check if we are in front of obstacle */
    if (dot(r_occ_vec, n_occ) > 0) {
        /* calculate the unit distance and gradient */
        
        /* update detection count */
        *occ_count += 1;
        
        /* distance to obstacle */
        const double r_occ = sqrt(dot(r_occ_vec,r_occ_vec));
        
        /* normalize ray vector (NOTE: flip (-) to point TOWARDS obsctacle) */
        r_occ_vec[0] = -r_occ_vec[0] / r_occ;
        r_occ_vec[1] = -r_occ_vec[1] / r_occ;
        r_occ_vec[2] = -r_occ_vec[2] / r_occ;
        
        /* relative velocity */
        const double vG_vec[3] = {vG_e, vG_n, -vG_d};
        double v_rel = dot(r_occ_vec, vG_vec); /* in ENU */
        if (v_rel < 0.0) v_rel = 0.0;
        const double v_rel_sq = v_rel*v_rel;
    
        /* radial buffer zone */
        const double delta_r = delta_r0 + v_rel_sq * k_delta_r;
    
        /* adjusted radial offset */
        const double r_offset_1 = r_offset + v_rel_sq * k_r_offset;
        
        /* calculate unit distance */
        const double r_unit = (r_occ - r_offset_1)/delta_r;
        
        /* calculate gradient */
        double jac_r_unit_temp[6];
        jacobian_r_unit(jac_r_unit_temp, 
            delta_r, gamma, k_delta_r, k_r_offset,
            n_occ[0], n_occ[2], n_occ[1],
            r_unit, v,
            r_occ_vec[0], r_occ_vec[2], r_occ_vec[1],
            v_rel, xi);
        
        /* update minimum unit distance */
        if ((r_unit < *r_unit_min) || *occ_count == 1) {
            *r_unit_min = r_unit;
            *f_min = (r_unit < 0.0);
        }
        
        /* add */
        jac_r_unit[0] += jac_r_unit_temp[0];
        jac_r_unit[1] += jac_r_unit_temp[1];
        jac_r_unit[2] += jac_r_unit_temp[2];
        jac_r_unit[3] += jac_r_unit_temp[3];
        jac_r_unit[4] += jac_r_unit_temp[4];
        jac_r_unit[5] += jac_r_unit_temp[5];
    }
}

/* cast a ray along the ground speed vector and return any detection point and normal */
void get_occ_along_gsp_vec(double *p_occ, double *n_occ, double *r_occ, int *occ_detected,
        const double *states, const double *speed_states, const double *terr_params,
        const double terr_local_origin_n, const double terr_local_origin_e,
        const int map_height, const int map_width, const double map_resolution, const double *terr_map)
{
    /* DEFINE INPUTS - - - - - - - - - - - - - - - - - - - - - - - - - - */
    
    /* states */
    const double r_n = states[0];
    const double r_e = states[1];
    const double r_d = states[2];
    const double v = states[3];
    const double gamma = states[4];
    const double xi = states[5];
    
    /* speed states */
    const double vG_sq = speed_states[6];
    const double vG_n_unit = speed_states[9];
    const double vG_e_unit = speed_states[10];
    const double vG_d_unit = speed_states[11];
    
    /* radial params */
    const double r_offset = terr_params[4];
    const double delta_r0 = terr_params[5];
    const double k_r_offset = terr_params[6];
    const double k_delta_r = terr_params[7];
    const double log_sqrt_w_over_sig1_r = terr_params[8];
    const double one_over_sqrt_w_r = terr_params[9];
    
    /* cast ray along ground speed vector to check for occlusions */
    
    /* init */
    double p1[3];
    double p2[3];
    double p3[3];
    
    /* ray vector */
    const double v_ray[3] = {vG_e_unit, vG_n_unit, -vG_d_unit};
    
    /* radial buffer zone */
    const double delta_r = delta_r0 + vG_sq * k_delta_r;
    
    /* adjusted radial offset */
    const double r_offset_1 = r_offset + vG_sq * k_r_offset;
    
    /* ray length */
    const double d_ray = delta_r + r_offset_1 + map_resolution;
    
    /* ray start ENU */
    const double r0[3] = {r_e, r_n, -r_d};
    /* ray end ENU */
    const double r1[3] = {r0[0] + v_ray[0] * d_ray, r0[1] + v_ray[1] * d_ray, r0[2] + v_ray[2] * d_ray};
    
    /* cast the ray */
    *occ_detected = castray(r_occ, p_occ, n_occ, p1, p2, p3, r0, r1, v_ray,
            terr_local_origin_n, terr_local_origin_e, map_height, map_width,
            map_resolution, terr_map);
    
    /* shift occlusion origin */
    p_occ[0] = p_occ[0] + terr_local_origin_e;
    p_occ[1] = p_occ[1] + terr_local_origin_n;
}