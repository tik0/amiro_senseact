
mu = [10 10];
n = 200;
N1 = randn(n,2) + repmat(mu,n,1);

mu = [-10 -10];
n = 200;
N2 = randn(n,2)* chol([1,0;0,10]) + repmat(mu,n,1);

mu = [10 12];
n = 200;
N3 = randn(n,2) + repmat(mu,n,1);

N = [N1 ; N2; N3]
% N = [N1 ; N2]

Nj = N(:,1) + 1j * N(:,2);

Nj_trans = transpose(Nj);

R = abs(repmat(Nj , 1, numel(N) / 2) - repmat(Nj_trans, numel(N) / 2, 1));

S = exp(-R);

L = ClusterGraph_Newman(S);

figure(1)
clf
hold on
c = linspace(1,100,max(L));
for idx = 1 : max(L)
    scatter(N(L == idx ,1), N(L == idx,2), [] ,repmat(c(idx), 1, sum(L==idx)))
end


%% Try online clustering
Nmu = zeros(max(L),1);
for idx = 1 : max(L)
    Nmu(idx) = mean(Nj(L == idx));
    scatter(real(Nmu(idx)), imag(Nmu(idx)), [] ,[ 1,0,0],'fill');
end


mu = [10 11];
n = 100;
N = randn(n,2)*chol([10,0;0,0.1]) + repmat(mu,n,1);

Nj = [Nmu; N(:,1) + 1j * N(:,2)];
Nj_trans = transpose(Nj);
R = abs(repmat(Nj , 1, (numel(N)) / 2 + max(L)) - repmat(Nj_trans, (numel(N)) / 2 +max(L), 1));

S = exp(-R);
S(1:max(L),1:max(L)) = eye(max(L));

Ln = ClusterGraph_Newman(S);

figure(2)
clf
hold on
c = linspace(1,100,max(Ln));
for idx = 1 : max(Ln)
    scatter(real(Nj(Ln == idx)), imag(Nj(Ln == idx)), [] ,repmat(c(idx), 1, sum(Ln==idx)))
end
