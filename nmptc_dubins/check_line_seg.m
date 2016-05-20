function bool = check_line_seg( pos, pparams )
    
    % calculate vector from waypoint a to b
    ab_n = pparams(6) - pparams(3);
    ab_e = pparams(7) - pparams(4);
    ab_d = pparams(8) - pparams(5);
    
    norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e + ab_d*ab_d );

    proj_w = ( pparams(20)*ab_n + pparams(21)*ab_e + pparams(22)*ab_d ) / norm_ab;

    % TODO: note this assumes hard-coded 30 [deg] max bank
    r_acpt = ( pparams(1) + proj_w ) * ( pparams(1) + proj_w ) / 5.6638;

    % 1-D track position
    pb_t = norm_ab - ( (ab_n*(pos(1)-pparams(3)) + ab_e*(pos(2)-pparams(4)) + ab_d*(pos(3)-pparams(5))) ) / norm_ab;

    % check
    bool = (pb_t - r_acpt) < 0.0;