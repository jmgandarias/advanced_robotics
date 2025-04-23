function q2=qinv(q)

    q2=[q(1)/(sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2)),...
        -q(2:4)/(sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2))];
    end