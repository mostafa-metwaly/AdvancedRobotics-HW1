function q = ikLeg(T_base, p_global, link, flag)
%     T_base - transform from global coordinate frame to local one of the leg
    R_base = T_base(1:3, 1:3);
    p_base = T_base(1:3, 4);
    p_local = R_base'*(p_global - p_base);

    x = p_local(1);
    y = p_local(2);
    z = p_local(3);

    cos_q2 = (x^2 + y^2 - link(1)^2 - link(2)^2)/(2*link(1)*link(2));
    sin_q2 = flag*sqrt(1 - cos_q2^2);
    q2 = atan2(sin_q2, cos_q2);
    q1 = atan2(y, x) - atan2(link(2)*sin(q2), link(1) + link(2)*cos(q2));
    q3 = -(q1 + q2);
    q = [q1, q2, q3];