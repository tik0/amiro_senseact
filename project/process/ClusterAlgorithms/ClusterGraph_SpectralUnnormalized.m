function l = ClusterGraph_SpectralUnnormalized(S)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File: SpectralClustering.m
%       Unsupervised Spectral Clustering, using eigengap for choosing k
% 
%       k:     Number of cluster
%       N:     Number of samples
%       mu:    1xC Mean values of the single distrebutions. C: Number of Dist.
%       sigma: Standart deviation of the single dist.
%       y:     C*Nx1 Concanation of all Distrebutions y1, y2, ...,yC
%
%       S:     C*NxC*N Graph representation or Similarity matrix
%       D:     C*NxC*N Degree of all vertecies on the diagonal
%       L:     C*NxC*N Laplacian of S
%       Y:     C*Nxk Concanation of the k eigenvector regarding every eigenvalue beneath the eigengap
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
NumData = size(S,1);

%S = S ./ max(max(S));                              % Normalization
S = S+S';                                           % Crate an undirected graph
%S = S>0;                                            % Make an epsilon graph

%% Calculate the Laplacian
D = diag(sum(S),0);
%DSqrt = sqrt(D);
L = D-S;% / DSqrt;

%% Calculate the spectrum (eigenvalues) of the Laplacian
[Vec,Val] = eig(L);
[ValSorted, ValIdx] = sort(sum(Val));       % Sort them ascending


%% Find the eigengap and assigne number of cluster k
% Find the gap by derivating the sorted eigenvalues and choosing the first local maxima
[~,eigengapIdx] = findpeaks(conv(round(ValSorted),[1 -1]));
if ~numel(eigengapIdx)
   [~,eigengapIdx] = max(conv(round(ValSorted),[1 -1]));
end
k = eigengapIdx(1)-1;

% if k>11
%     k = 11;
% end

% Plot the eigengap
%fprintf('Dim. Nullraum %i, Number of Nodes %i, Eigengap at %i\n',NumData-rank(L),NumData,k);
% figure(11111)
% stem(ValSorted); grid;
% title('The first ten eigenvalues sorted ascending its size')
% hold on
% line([1 size(ValSorted,1)],(ValSorted(k+1)+ValSorted(k))/2*ones(1,2),'Color','r','LineStyle','--')

pause

%% Choose the k eigenvector regarding the smallest eigenvalues
Y = zeros(NumData,k);
for Idx = 1 : k
    Y(:,Idx) = transpose(Vec(:,ValIdx(Idx)));
end

%% Cluster the eigenvectors and assigne the labels
DoKmeans = 1;
while(DoKmeans)
    try
        l = kmeans(Y,k,'replicates',10);
        DoKmeans = 0;
    catch
        % If no replicate has found a proper cluster, try it again
        k = k-1;
        Y = Y(:,1:end-1);
        if k>0
            warning('K-Means suxxs, trying again with k-1=%i cluster',k);
        else
            warning('Choosing only one cluster');
            l = ones(NumData,1);
            return;
        end
    end
end