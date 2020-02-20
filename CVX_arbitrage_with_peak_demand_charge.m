clear 
close all
clc
load('real_time_data')
tic
pts = 96; %%number of points considered
time=real(1:pts,2);
price=real(1:pts,1);

load('15_min_load_data.mat')
load('15_min_pv_data.mat')

P_g = avg_active_grid(1:pts,1);
Q_g = avg_reactive_grid(1:pts,1);

lambda_p = 1;

e_ch=0.95;
e_dis =0.95;
del_max = 2000;
del_min = -del_max;
B_0 = 1000*ones(pts,1);
B_max = 2000*ones(pts,1);
B_min = 200*ones(pts,1);
h=0.25;
x_upper= del_max*h*ones(pts,1);
x_lower= del_min*h*ones(pts,1);
S_B_max = 1*del_max*h/e_ch;

M = tril(ones(pts,pts));

cvx_begin 
variables x_ch(pts,1) x_ds(pts,1) 
variables b_p_bat(pts,1)
minimize sum(price'*x_ch/e_ch-price'*x_ds*e_dis)       %trace((x_ch/e_ch-x_ds*e_dis)*price')
subject to 
    zeros(pts,1)<= x_ch <= x_upper;    %%charging 
    zeros(pts,1)<= x_ds <= -x_lower;    %% discharging 
    b_p_bat == x_ch-x_ds;
    B_min <= B_0 + M*b_p_bat <= B_max;  %%battery capacity
%     x_ch/e_ch-x_ds*e_dis + P_g >= zeros(pts,1);
cvx_end
profit_only_arbitrage =  sum(price'*x_ch/e_ch-price'*x_ds*e_dis)/1000
B = B_0 + M*b_p_bat;
b_p = x_ch/e_ch-x_ds*e_dis;

%%
cvx_begin 
variables x_ch(pts,1) x_ds(pts,1) 
variables b_p_bat(pts,1)
% variable theta
minimize sum(price'*x_ch/e_ch-price'*x_ds*e_dis) + lambda_p*max(x_ch/e_ch-x_ds*e_dis + P_g)       %trace((x_ch/e_ch-x_ds*e_dis)*price')
subject to 
    zeros(pts,1)<= x_ch <= x_upper;    %%charging 
    zeros(pts,1)<= x_ds <= -x_lower;    %% discharging 
%     theta == lambda_p*max(x_ch/e_ch-x_ds*e_dis + P_g);
    b_p_bat == x_ch-x_ds;
    B_min <= B_0 + M*b_p_bat <= B_max;  %%battery capacity
%     x_ch/e_ch-x_ds*e_dis + P_g >= zeros(pts,1);
cvx_end
profit_arbitrage_peak_demand =  sum(price'*x_ch/e_ch-price'*x_ds*e_dis)/1000
B_pd = B_0 + M*b_p_bat;
b_p_peak_demand = x_ch/e_ch-x_ds*e_dis;



figure
plot(P_g)
hold on
plot(b_p+P_g)
hold on
plot(b_p_peak_demand + P_g)

abs(profit_arbitrage_peak_demand)/abs(profit_only_arbitrage)