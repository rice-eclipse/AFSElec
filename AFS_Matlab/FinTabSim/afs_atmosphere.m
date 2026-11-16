function [rho, a, p, Temp, g] = afs_atmosphere(P, h)
%AFS_ATMOSPHERE ISA troposphere/lower stratosphere at height h above the pad.
hA = max(h + P.env.hSite, 0);
if hA < 11000
    Temp = 288.15 - 0.0065*hA;
    p    = 101325*(Temp/288.15)^5.25588;
else
    Temp = 216.65;
    p    = 22632.1*exp(-9.80665*(hA - 11000)/(287.053*216.65));
end
rho = p/(287.053*Temp);
a   = sqrt(1.4*287.053*Temp);
g   = P.env.g0*(P.env.Re/(P.env.Re + hA))^2;
end
