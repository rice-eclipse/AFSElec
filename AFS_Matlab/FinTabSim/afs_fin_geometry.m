function [r, t] = afs_fin_geometry(P, i, xCg)
%AFS_FIN_GEOMETRY Position of fin i's center of pressure and its force direction.
%   r : vector from the CG to the fin CP (body frame, m)
%   t : unit normal of the fin plane; a positive tab force acts along +t
phi = P.geo.finAngle(i);
er  = [0; cos(phi); sin(phi)];          % radial direction of the fin
t   = [0; -sin(phi); cos(phi)];         % = cross([1;0;0], er)
r   = [-(P.geo.xFin - xCg); 0; 0] + P.geo.rFin*er;  % body x points forward
end
