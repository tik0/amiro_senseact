function [Q] = ClusterGraph_Voronoi(SeedNodeArray, ClusterSeed, NumNodes)
%ClusterGraph_Newman Clustering algorithm of M. E. J. Newman 
%   By Timo <korthals@mail.upb.de>: 
%
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

NumSeeds = numel(ClusterSeed);

%% Creating a similarity matrix
S = zeros(NumNodes);
for SeedIdx = 1 : NumSeeds
    SeedNodeArray = SeedNodeArray ./ max(max(SeedNodeArray));       % Normalize, so that the worst value is 1
    S(ClusterSeed(SeedIdx),:) = 1-SeedNodeArray(:,SeedIdx);     % Creating similarity, while close to 1 means greate similarity
end
% Deleting the diagonal
S(sub2ind(size(S),1:NumNodes,1:NumNodes)) = zeros(1,NumNodes);
% Creating a full similarity matrix
S = S + S';


%% Calculate the clusterassociation to the clusterseed
[~,Nodelabels] = min(SeedNodeArray,[],2);

%% init
% Normalize the matrix, so that the sum over all nodes is 1
%S = S > 0; % Make an unweighted graph
S = S / sum(S(:));

% Joins = zeros(NumNodes-1,2);    % initalize array for joins

%dQmax = -Inf(NumNodes-1, 1);    % maximum change in modularity at each step
Q = zeros(NumNodes+1, 1);         % modularity at each step
Q(1) = CalcMod(S);              % inital modularity

%% Calculation of the groups
ModGrp = 1;         % Seperate counter for the modularity
for NodeIdx = 1:NumNodes%-NumSeeds
    %% Calculation of deltaQ
%     dQ = -Inf(size(S));
%     Grp_found = 0;
%     for Idx_i = 1:size(S,1)-1
%         for Idx_j = Idx_i+1:size(S,1)
%             if S(Idx_i,Idx_j) ~= 0
%                 a_i = sum(S(Idx_i,:));
%                 a_j = sum(S(:,Idx_j));
%                 dQ(Idx_i,Idx_j) = 2*(S(Idx_i,Idx_j) - a_i*a_j);
%                 if(dQmax(IdxGrp) < dQ(Idx_i,Idx_j))
%                     dQmax(IdxGrp) = dQ(Idx_i,Idx_j);
%                     Grp_1 = Idx_i;
%                     Grp_2 = Idx_j;
%                     Grp_found = 1;
%                 end
%             end
%         end
%     end
    
    %% Calculate the next group to join
    
 %   while 1
        Grp_1 = ClusterSeed(Nodelabels(NodeIdx));       % Get the seed
        Grp_2 = NodeIdx;%Nodelabels(IdxGrp);                     % Get the node
        % Wath out, that the groups are not the same (This can only happen wenn we want to group the same seed with itself)
        if (Grp_1 == Grp_2)
            continue;%break;
        end
        ModGrp = ModGrp + 1;
 %   end

    %Joins(IdxGrp,:) = [Grp_1, Grp_2];

    
    %% Update the similarity matrix
    % Update of the groupweights e_ii and e_jj
    e_ii_old = S(Grp_1, Grp_1);
    e_jj_old = S(Grp_2, Grp_2);
    e_ij = S(Grp_1, Grp_2);
    e_ii_new = e_ii_old + e_jj_old + 2*e_ij;
    S(Grp_1, Grp_1) = e_ii_new;
    
    % Update of all the other groupweights, which have been connected to the
    % found group
    delta_S = zeros(size(S));
    for Idx_i = 1:size(S, 1)
        if ~((Idx_i == Grp_1) || (Idx_i == Grp_2))
            delta_S(Idx_i,Grp_1) = delta_S(Idx_i,Grp_1) + S(Idx_i,Grp_2);
        end
    end
    S = S + delta_S + delta_S';

    % Removing of the unused group j (j -> i)
    S(:,Grp_2) = zeros(size(S(:,Grp_2)));
    S(Grp_2,:) = zeros(size(S(Grp_2,:)));

    % Update of the modularity
    Q(ModGrp) = CalcMod(S);%Q(IdxGrp) + dQ(Grp_1,Grp_2);
end

spy(sparse(S))