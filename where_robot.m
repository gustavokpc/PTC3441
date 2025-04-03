function [ trels ] = where_robot( theta, trelw, srelb, L )
%WHERE_ROBOT Summary of this function goes here
%   Detailed explanation goes here
brels = tinvert(srelb);
wrelb = kin(theta, L);

trels_t = tmult(brels, wrelb);
trels = tmult(trels_t, trelw);

end

