function [Gamma] = christoffel_symbols(M, q, n)
% christoffel_symbols computes the Christoffel symbols of a given system
% represented by the mass matrix M, the generalized coordinate vector q and
% the number of degrees of freedom 'n'
    for i = 1:n
        for j = 1:n
            for k = 1:n
                Gamma(i, j, k) = 1/2 * (diff(M(i,j), q(k)) + diff(M(i, k), q(j)) - diff(M(j, k), q(i)));
            end
        end
    end
end

