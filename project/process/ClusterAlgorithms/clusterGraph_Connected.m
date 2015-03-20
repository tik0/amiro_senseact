function l = clusterGraph_Connected(A)
%clusterGraph_connected gives the cluster of cliques in graph.
%           A node N correspondes to clique C, if any of the nodes N_C in the
%           clique C is reachable by N.
%           Definition of reachability:
%           If there is a chain: V1->E1->V2->E2->V3 then
%           V1 and V2 are DIRECT REACHABLE via E1, and V1 and V3 are REACHABLE via E1->V2->E2.
%           Algorithm:
%           Using bottom-up hirachical clustering, startin with the first node.
%           Then adding every connected node to the startingnode by a Depth-first Search.
%
%   Input: S NxN Adijacency matrix for N nodes with values in the upper right trianglar
%           1 for a connected, 0 for none
%
%   Output: l Nx1 Labellist for N nodes
    
%% Init
    % Make A to the needed structure
    A = triu(A);
    A = A > 0;
    % Init the labellist
    l = zeros(size(A,1),1);
    
%% Calculation
    % Grouping the nodes. There can be n groups at it's maximum
    for GrpIdx = 1 : size(A,1)
        % Get the first row/node, which is not grouped yet
        l_notGrouped = find(l==0,1);
        % Call the function with the first element, which is not grouped
        l = FindConnections(GrpIdx,l_notGrouped,A,l);
        % Check if everything is grouped. Otherwise go on with grouping
        if isempty(find(l==0, 1))
            break;
        end
    end
     

end

function l = FindConnections(Grouplabel,NodeToGroup,A,l)
%FindConnections Recursive function which outputs the updated labellist
%               with all nodes labled the same GrpIdx, which are connected
%               to NodeToGroup
%
%   Input: Grouplabel Grouplabel which will be assigned to NodesToGroup
%          NodeToGroup Nodeindexes, which gets the label Grouplabel
%          A NxN Adjiacency matrix with N nodes
%          l Nx1 Old labellist for N nodes
%
%   Output: l Nx1 Updated labellist for N nodes

%% Recursive call condition
    % Check if there are nodes connected to the actual node 
    if (find(A(NodeToGroup,:),1))
        % Iterate through the connected nodes
        for NodeIdx = 1 : sum(A(NodeToGroup,:))
            % Get the index of the nodes, which are connected to the actual node
            NodesToAdd = find(A(NodeToGroup,:));
            % Start an recursive call with one of the nodes, which are connected to the actual one.
            % But check first, if the node has no label yet, otherwise we have been here
            if (~l(NodesToAdd(1,NodeIdx)))
                l = FindConnections(Grouplabel,NodesToAdd(1,NodeIdx),A,l);
                %idx=idx+1;
            end
        end
    end
%% Return of labled node
    l(NodeToGroup) = Grouplabel;
end