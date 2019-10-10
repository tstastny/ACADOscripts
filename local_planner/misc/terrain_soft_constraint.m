clear;
clc;
close all;

x_set = linspace(0,1,101)';

iset = 1./[0.001 0.01 0.1];
jset = [1 10 100];
for i = 1:length(iset)
    for j = 1:length(jset)
        sig_h_exp(:,i,j) = jset(j)*exp(-x_set*log(jset(j)*iset(i)));
    end
end
%%
figure('color','w'); hold on; grid on;
j=1;
for i = 1:length(iset)
    plot(x_set, sig_h_exp(:,i,j))
end

xlim([0 1])
ylim([0 1])
%%
figure('color','w'); hold on; grid on;
i=1;
for j = 1:length(jset)
    plot(x_set, sig_h_exp(:,i,j))
end

xlim([0 1])
ylim([0 100])