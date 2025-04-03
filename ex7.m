L = [0.5 0.3];

trelw = utoi([0.1 0.2 30.0]);
srelb = utoi([-0.1 0.3 0.0]);

theta_1 = [0.0 90.0 -90.0];
theta_2 = [-23.6 -30.3 48.0];
theta_3 = [130.0 40.0 12.0];

trels_1 = where_robot(theta_1, trelw, srelb, L);
trels_2 = where_robot(theta_2, trelw, srelb, L);
trels_3 = where_robot(theta_3, trelw, srelb, L);
