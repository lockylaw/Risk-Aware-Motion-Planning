function SP_num = SenarioProgSampNum(beta, s_hut, sigma, varargin)
% refer to paper : a general scenario theory for nonconvex optimization and
% dicision making

% input: beta   --- confidence parameter
%        s_hut  --- upper bound
%        sigma  --- collison 
%        R(varargin) --- discarding number

SP_num = 100;
if ~isempty(varargin)
    R = varargin(1);
else
    R = 0 ;
end
epsilon = 1;

while epsilon > sigma
    S = SP_num;
    P = S - R;
    beta_trans = beta / nchoosek(S, P);
    epsilon = 1 - (beta_trans / (P * nchoosek(P, s_hut)))^(1/(P-s_hut));
    SP_num = SP_num + 50;
end

