SP_num = 1000;
R = 50;
s_hut = 20;
beta = 1e-6;
epsilon = 1;

while epsilon > 1 -0.9889
    S = SP_num;
    P = S - R;
    beta_trans = beta / nchoosek(S, P);
    epsilon = 1 - (beta_trans / (P * nchoosek(P, s_hut)))^(1/(P-s_hut));
    SP_num = SP_num + 50;
end

% plot(1:500, epsilon)