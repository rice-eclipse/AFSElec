function B = afs_tab_effectiveness(P, xCg)
%AFS_TAB_EFFECTIVENESS Body moment per unit tab deflection per unit dynamic pressure.
%   B = AFS_TAB_EFFECTIVENESS(P, xCg) returns a 3xN matrix. Column i is the
%   moment [L; M; N] (N m) produced by tab i at 1 rad and qbar = 1 Pa, with the
%   CG at station xCg. A positive tab pushes its fin along t_i = x_b x e_r.
n = P.geo.nFins;
B = zeros(3, n);
for i = 1:n
    [r, t] = afs_fin_geometry(P, i, xCg);
    B(:, i) = P.geo.Sref*P.aero.CNdTab*cross(r, t);
end
end
