
iset = 9; % choose objectives (dy/)
jset = 1:3; % choose states (/dx)

len_tt = size(rec.dydx,2);

for i = iset
    for j = jset
    
        figure('color','w','Name',['d(y',int2str(i),')/d(x',int2str(j),')']);
        hold on; grid on; box on;

        surf(Ts_nmpc*((1:len_tt)'-1), (1:N)', rec.dydx(:,:,(i-1)*n_X+j));

        ylabel('N');
        xlabel('Time [s]');
        zlabel(['d(y_{',int2str(i),'})/d(x_{',int2str(j),'})']);
    end
end

% for i = 1:12
%     
%     yhor = rec.y(:,:,i);
%     
%     figure('color','w','Name',['y',int2str(i)]);
%     hold on; grid on; box on;
%     
%     surf(Ts_nmpc*((1:size(yhor,2))'-1), (1:N)', yhor);
%     
%     ylabel('N');
%     xlabel('Time [s]');
%     zlabel(['y_',int2str(i)]);
%     
% end