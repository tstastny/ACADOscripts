% / / / / / / / / / / / / / / / / / / / /
% EXPONENTIAL SOFT CONSTRAINT FORMULATION
% / / / / / / / / / / / / / / / / / / / /

clear;
clc;
close all;

x_off = 0;
x_set = linspace(x_off-1,x_off+1,101)';

iset = 1./[0.001 0.01 0.1]; % these are the sig1's
jset = [1 10 100]; % these are the weights applied externally
for i = 1:length(iset)
    for j = 1:length(jset)
        normalized_state = x_set - x_off;
%         sig_h_exp(normalized_state<0,i,j) = jset(j)*(sqrt(1-normalized_state(normalized_state<0)*log(jset(j)*iset(i)))).^2;
%         sig_h_exp(normalized_state>=0,i,j) = jset(j)*(sqrt(exp(-normalized_state(normalized_state>=0)*log(jset(j)*iset(i))))).^2;
        sig_h_exp(normalized_state<0,i,j) = jset(j)*(1-normalized_state(normalized_state<0)*log(sqrt(jset(j)*iset(i)))).^2;
        sig_h_exp(normalized_state>=0,i,j) = jset(j)*(exp(-normalized_state(normalized_state>=0)*log(sqrt(jset(j)*iset(i))))).^2;
%         sig_h_exp(normalized_state>0,i,j) = jset(j)*(1+normalized_state(normalized_state>0)*log(sqrt(jset(j)*iset(i)))).^2;
%         sig_h_exp(normalized_state<=0,i,j) = jset(j)*(exp(normalized_state(normalized_state<=0)*log(sqrt(jset(j)*iset(i))))).^2;
    end
end

%%
figure('color','w'); hold on; grid on;
j=1;
i=1;%for i = 1:length(iset)
    plot(x_set, sig_h_exp(:,i,j))
% end

xlim(x_set([1 end]))
ylim([0 10])

%%
figure('color','w'); hold on; grid on;
i=1;
for j = 1:length(jset)
    plot(x_set, sig_h_exp(:,i,j))
end

xlim(x_set([1 end]))
ylim([0 100])

%%
figure('color','w'); hold on; grid on;
for i = 1:length(iset)
    for j = 1:length(jset)
        plot(x_set, sig_h_exp(:,i,j))
    end
end

xlim(x_set([1 end]))
ylim([0 100])