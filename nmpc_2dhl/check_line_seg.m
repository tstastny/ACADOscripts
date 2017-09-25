function bool = check_line_seg( pos, pparams )
    
    % calculate vector from waypoint a to b
    ab_n = pparams(4) - pparams(1);
    ab_e = pparams(5) - pparams(2);
    
    norm_ab = sqrt( ab_n*ab_n + ab_e*ab_e );
    
    % 1-D track position
    pb_t = norm_ab - ( ab_n*(pos(1)-pparams(1)) + ab_e*(pos(2)-pparams(2)) ) / norm_ab;

    % check
    bool = pb_t < 0.0;