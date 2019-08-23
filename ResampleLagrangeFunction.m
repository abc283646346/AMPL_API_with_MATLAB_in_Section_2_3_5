function y = ResampleLagrangeFunction(x0, y0, x)
n = length(x0);
m = length(x);
y = zeros(1,m);

for ii = 1 : m
    z = x(ii);
    s = 0;
    for kk = 1 : n
        p = 1;
        for jj = 1 : n
            if jj ~= kk
                p = p * (z - x0(jj)) / (x0(kk) - x0(jj));
            end
        end
        s = p * y0(kk) + s;
    end
    y(ii) = s;
end