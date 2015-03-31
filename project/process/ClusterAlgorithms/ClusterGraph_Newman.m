function [nodelabels] = ClusterGraph_Newman(S,varargin)
%ClusterGraph_Newman Clustering algorithm of M. E. J. Newman 
%   By Timo <korthals@mail.upb.de>: Bugfix: If there is no new Group, the last Q value must be deletet, because it has no new information (infact otherwise it leads to an error)
%
%   Usage:
%   ClusterGraph_Newman(S)                 : Cluster the graph S regarding the maximized modularity Q
%   ClusterGraph_Newman(S,'Q',90)          : Cluster the graph S regarding the 90 percent of maximized modularity Q
%   ClusterGraph_Newman(S,'Cluster',30)    : Cluster the graph S with 30 cluster
%   
%   
%   Reprogramming of the paper with the "Fast algorithm for detecting 
%   community structure in networks".
%   The algorithm is optimizing the modularity Q by trying to maximize it.
%   It goups two nodes i and j to i, which increases Q and then recalculate
%   all the weights wich have been affected by this grouping.
%
%   INPUT:
%       S:              NxN Similarity matrix
%       varargin:       See Usage!
%
%   OUTPUT:
%       nodelabels: Nx1 vector containing grouplabels for each node

%% init
% Read out the varargin variable
NumArgIn = nargin-1;
if (NumArgIn == 0)
    disp('Using the segmentation of the maximized modularity Q');
    NumClasses = [];
    Q_decrease = [];
elseif (NumArgIn == 2)
    if strcmp(varargin{1},'Q')
        Q_decrease = varargin{2};
        NumClasses = [];
    elseif strcmp(varargin{1},'Cluster')
        NumClasses = varargin{2};
        Q_decrease = [];
    else
        error('Using "Q" or "Cluster" as argument')
    end
else
    error('See the comments for correct usage!')
end

% Normalize the similarity matrix, so that the sum over all elements is 1 (is used for Newmann cluster alhorithm)
S = (S / sum(S(:)));

NumNodes = size(S,1);           % number of vertices
Joins = zeros(NumNodes-1,2);    % initalize array for joins

dQmax = -Inf(NumNodes-1, 1);    % maximum change in modularity at each step
Q = zeros(NumNodes, 1);         % modularity at each step
Q(1) = CalcMod(S);              % inital modularity

%% Calculation of the groups
for IdxGrp = 1:NumNodes-1
    % fprintf('%d\n',IdxGrp);
    %% Calculation of deltaQ and the group to join   
    a_i = sum(S,2);
    a_j = sum(S,1);
    dQ = triu(2*(S - a_i*a_j),1);
    [Idx_i,Idx_j,dQValues] = find(triu(dQ,1));
    [dQmax(IdxGrp),I] = max(dQValues);
    Grp_1 = Idx_i(I);
    Grp_2 = Idx_j(I);
    
    %% Add the group to the Joinsvector
    Joins(IdxGrp,:) = [Grp_1, Grp_2];

    %% Update
    % Update of the groupweights e_ii and e_jj
    e_ii_old = S(Grp_1, Grp_1);
    e_jj_old = S(Grp_2, Grp_2);
    e_ij = S(Grp_1, Grp_2);
    e_ii_new = e_ii_old + e_jj_old + 2*e_ij;
    S(Grp_1, Grp_1) = e_ii_new;

    % Update of all the other groupweights, which have been connected to the
    % found group
    Mask = (1:size(S,1) ~= [Grp_1]) & (1:size(S,1) ~= [Grp_2]);
    S(Mask,Grp_1) = S(Mask,Grp_1) + S(Mask,Grp_2);
    S(Grp_1,Mask) = S(Grp_1,Mask) + S(Grp_2,Mask);

    % Removing of the unused group j (j -> i)
    S(:,Grp_2) = zeros(size(S(:,Grp_2)));
    S(Grp_2,:) = zeros(size(S(Grp_2,:)));

    % Update of the modularity
    Q(IdxGrp + 1) = Q(IdxGrp) + dQ(Grp_1,Grp_2) + dQ(Grp_2,Grp_1);

    % Check if there are any nodes left for grouping
    if (triu(S,1)==0)
        break;
    end
end

%% Save the grouplabels (1,2,3,...,NumGroups)

    if ~isempty(NumClasses)
        IdxMaxQ = NumNodes - NumClasses + 1;
    elseif ~isempty(Q_decrease)
        IdxMaxQ = find( Q >= (max(Q)*Q_decrease/100),1,'first');        % Take the partition wich has 90% of the maximal modularity Q
    else
        [~, IdxMaxQ] = max(Q);
    end

    NumGroups = NumNodes - IdxMaxQ + 1;
    nodelabels = zeros(NumNodes,1);
    
    % Going from the top to the bottom in the dendrogram
    GroupsIdx = 0;
    if (exist('Joins','var'))
        for JoinsIdx = IdxMaxQ-1 : -1 : 1
            if ( ( nodelabels(Joins(JoinsIdx,1)) == 0 ) && ( nodelabels(Joins(JoinsIdx,2)) == 0 ) )
                GroupsIdx = GroupsIdx + 1;                  % Increase the grouplabel
                nodelabels(Joins(JoinsIdx,1)) = GroupsIdx;  % Assigne the new label
                nodelabels(Joins(JoinsIdx,2)) = GroupsIdx;  % Assigne the new label
            elseif ( ( nodelabels(Joins(JoinsIdx,1)) ~= 0 ) && ( nodelabels(Joins(JoinsIdx,2)) == 0 ) )
                nodelabels(Joins(JoinsIdx,2)) = nodelabels(Joins(JoinsIdx,1)); % Assigne the label from the one group to another
            elseif ( ( nodelabels(Joins(JoinsIdx,2)) ~= 0 ) && ( nodelabels(Joins(JoinsIdx,1)) == 0 ) )
                nodelabels(Joins(JoinsIdx,1)) = nodelabels(Joins(JoinsIdx,2)); % Assigne the label from the one group to another
            else
                error('Cannot join two nodes with different grouplabels.');
            end
        end
    end
    
    % If there are nodes left without label, then they get there own one
    for nodelabelsIdx = 1 : size(nodelabels,1)
        if ( nodelabels(nodelabelsIdx) == 0 )
            GroupsIdx = GroupsIdx + 1;
            nodelabels(nodelabelsIdx) = GroupsIdx;
        end
    end

    % plot(Q)

    % Check if the number of groups is correct
%     if ( GroupsIdx > NumGroups )
%         error('To few grouplabels for to many groups.');
%     end