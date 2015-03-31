function [nodelabels] = ClusterGraph_Markov(S)
%ClusterGraph_Markov Clustering algorithm by van Dongen
%
%   INPUT:
%       S: NxN Similarity matrix
%
%   OUTPUT:
%       nodelabels: Nx1 vector containing grouplabels for each node

%% init
e = 2;               % Expansionfactor
r = 1.2;             % Inflationfactor
maxiters = 300;      % Iterations
minChange = 10^(-6); % Exit condition if the change of the matrix is smaller than this value

%% Calculation of the groups

% Add an edge to the node itself
%M = (S + diag(sum(S)./sum(S>0)));
M=S+diag(sum(S)==0);    % Adding Selfloops to Nodes with no edges
%S = S>0;               % Making an Adjiacencygraph out of the data
%M = S+eye(size(S));    % Adding selfloops


% Calculate the propability similarity matrix
%1./sum(M)
d = diag(1./sum(M));
M = M * d;
%M(isnan(M)) = 0;

% Iteration
i = 1;
% Calculate until the maximum itartions are reached or the changes of the
% matrix are to small
while ( ( i < maxiters ) && ( sum(sum(abs(M - M^2))) > minChange ) ),


    % Expansion
    M = M ^ e;

    % Inflation
    M = M .^ r;
    d = diag(1./sum(M));
    M = M * d;

    
    i = i + 1;
end

%% Save the grouplabels (1,2,3,...,NumGroups)

    % One node is only allowed to have one node. The group with the highest 
    % propability will be saved, all the other will be pruned
    [~,nodelabelsTmp] = max(M,[],1);
    
%% Assigne proper classlabels to the nodes
% The problem with the upper section is, that the classlabels are regarding the columns of the matrix.
% So they are not incremental, what is not allowed by the following code.
Classlabel = 1;
nodelabels = zeros(size(nodelabelsTmp));
% As long as not every value in the temporary vector is handeld, the relabeling will go on
while (min(nodelabelsTmp) ~= Inf)
    % Search the next minimum label in the temporary vector
    NextMinimumClassLabel = min(nodelabelsTmp);
    % Assigne these values a proper classlabel
    nodelabels(nodelabelsTmp == NextMinimumClassLabel) = Classlabel;
    % Increment the proper classlabel
    Classlabel = Classlabel + 1;
    % Set the temporary labels to Inf, so that they wont be handeled any more
    nodelabelsTmp(nodelabelsTmp == NextMinimumClassLabel) = Inf;
end