%% Coupled Equation Definition

function firstODE = coupledEqtns(t,y,m,J,kf,kr,L1,L2,cf,cr,radFreq,A,L)
% Function determines first order ODE equations from second order

firstODE = [y(2);
    (1/m)*(-(cf+cr)*y(2)-(kf+kr)*y(1)-(cr*L2-cf*L1)*...
    y(4)-(kr*L2-kf*L1)*y(3)+kf*A*sin((radFreq)*t)+kr*A*sin((radFreq)...
    *t-(2*pi*(L1+L2))/L));
    y(4);
    (1/J)*(-(cf*L1^2+cr*L2^2)*y(4)-(kf*L1^2+kr*L2^2)*y(3)-(cr*L2-cf*...
    L1)*y(2)-(kr*L2-kf*L1)*y(1)+kr*L2*A*sin((radFreq)*t-(2*pi*(L1+L2))/L)...
    -kf*L1*A*sin((radFreq)*t))];
end
