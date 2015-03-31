function [nodelabels] = ClusterGraph_Spectral(S)
%ClusterGraph_Spectral Clustering algorithm of M. E. J. Newman 
%   Reprogramming of the paper "On Spectral Clustering Analysis and an algorithm"
%   Use the similarity matrix as the afinity matrix
%
%   INPUT:
%       S: NxN Similarity matrix
%
%   OUTPUT:
%       nodelabels: Nx1 vector containing grouplabels for each node

%% init

k = 20;                         % Number of cluster

%% Add total affinity to the node itself
S = S + eye(size(S));

%% Calculation of the groups
D = diag(sum(S));
D_sqrt = sqrt(D);
L = D_sqrt \ S / D_sqrt;

[eigVector, eigValue] = eig(L);
% transpose(sum(eigValue))
% 
% max(eigValue)

% get the k greatest eigenvalues
maxEigValueIdx = -Inf(k,1);         % Contains the k maximum eigenvalues descending from Idx=1

for Idx = 1 : k
   [~, maxEigValueIdx(Idx)] = max(max(eigValue));
   eigValue(maxEigValueIdx(Idx),maxEigValueIdx(Idx)) = -Inf;
end

% Get the stacked matrix of the greatest eigenvectors
X = zeros(size(S,1),k);
for Idx = 1 : k
   X(:,Idx) = eigVector(:,maxEigValueIdx(Idx));
end

% Normalizing the rows of X
Y = zeros(size(S,1),k);
for Idx = 1 : size(S,1)
   normalization = ones(1,k) ./ sqrt(sum(X(Idx,:).^2));
   if (sum(isfinite(normalization) == 0) == 0) % Check if anything is NaN or Inf
        Y(Idx,:) = X(Idx,:) .* normalization;
   end
end

rank(Y')
pause

% Do k-means and save the labels
doKmeans = 1;
while(doKmeans)
    try
        [nodelabels,~] = kmeans(Y,k);%,'start','cluster','replicates',10);
        doKmeans = 0;
    catch
        k = k-1;
        if k == 0
            error('k=0')
        end
        warning('Decreasing of clustersize to %i',k);
    end
end
nodelabels = nodelabels';