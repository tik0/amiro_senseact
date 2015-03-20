function [ Q ] = CalcMod( R )
%CALCMOD Berechnung der Modularitaet
%
%   Input:
%       R: Adjazenzmatrix (gewichtet oder ungewichtete kanten)
%
%   Output:
%       Q: Modularitaet

Q = 0;
for Idx = 1:size(R,1)
    a_i = sum(R(:,Idx));
    e_ii = R(Idx,Idx);
    Q = Q + (e_ii - a_i^2);
end
end