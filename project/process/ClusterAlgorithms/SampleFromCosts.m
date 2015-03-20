function Clusterseed = SampleFromCosts(d_min)
%% SampleFromDist Drawing a sample of the propabilitydistribution A(S_t,:)
%    Input:     d_min               The vector with the minimum distances
%
%    Output:    Clusterseed         The next clusterseed



% Calculating the cumulativ sum
CumSum = cumsum(d_min);

% Get a random number between 0 and CumSum(end)

Random = rand()*CumSum(end);

% Find the first value, which is greater than the random value is the next seed
Clusterseed = find(CumSum >= Random, 1);

end