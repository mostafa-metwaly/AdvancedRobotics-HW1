function q = ikTripteron(T_base, p_global, link, flag)
    q = [];
    for leg = 1:size(T_base,3)
        q_leg = ikLeg(T_base(:,:,leg), p_global, link, flag);
        q(size(q,1)+1,:) = q_leg;
    end