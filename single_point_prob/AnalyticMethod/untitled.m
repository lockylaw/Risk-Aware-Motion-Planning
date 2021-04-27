SP_num = 500;
k = 0:SP_num;
beta = 1e-6;
s_hut = 20;
epsilon = zeros(SP_num+1, 1);
for i = 0:SP_num
    if k(i+1) < s_hut
        epsilon(i+1) = 1 - (beta / (s_hut * nchoosek(SP_num, k(i+1))))^(1/(SP_num-k(i+1)));
    else
        epsilon(i+1) = 1;
    end
end

plot(k, epsilon)