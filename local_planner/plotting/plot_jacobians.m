
iset = 1:3; % choose objectives (dy/)
j=6; % choose state (/dx)


dy_dx = rec.dydx(:,:,6:n_X:end); 

for i = iset
    
    figure('color','w','Name',['d(y',int2str(i),')/d(x)']);
    hold on; grid on; box on;
    
    surf(Ts_nmpc*((1:size(dy_dx,2))'-1), (1:N)', dy_dx(:,:,i));
    
    ylabel('N');
    xlabel('Time [s]');
    zlabel(['d(y_{',int2str(i),'})/d(x)']);
    
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