function Q = Q_from_P(P, dt)
assert (size(P, 1) == size(P, 2))
N = size(P, 1);
Q = zeros(N, N);
for i=1:N
    for j=1:N
        if j==i
            continue
        end
        Q(i, j) = P(j, i)/dt;
    end
    Q(i, i) = -sum(Q(i, :));
end

end