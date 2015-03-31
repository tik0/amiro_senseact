function [nodelabels] = ClusterGraph_BronKerbosch(S)
%ClusterGraph_Newman Clustering algorithm of M. E. J. Newman 
%   Reprogramming of the paper with the "Fast algorithm for detecting 
%   community structure in networks".
%   The algorithm is optimizing the modularity Q by trying to maximize it.
%   It goups two nodes i and j to i, which increases Q and then recalculate
%   all the weights wich have been affected by this grouping.
%
%   INPUT:
%       S: NxN Similarity matrix
%
%   OUTPUT:
%       nodelabels: Nx1 vector containing grouplabels for each node

%% init

% Get the upper triangular of the matrix
S = triu(S);

% Make the matrix to an unweighted graph
S > 0;

nodelabels = maximalCliques(S);