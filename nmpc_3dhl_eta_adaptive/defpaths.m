% /////////////////////////////////////////////////////////////////////////
% dubins paths list ///////////////////////////////////////////////////////

% pparam1 | type    type
% pparam2 | aa_n    cc_n
% pparam3 | aa_e    cc_e
% pparam4 | aa_d    cc_d
% pparam5 | bb_n    R
% pparam6 | bb_e    dir
% pparam7 | bb_d    gam
% pparam8 | --      xi0
% pparam9 | --      dxi

% /////////////////////////////////////////////////////////////////////////


% paths(1).pparam1 = 1;
% paths(1).pparam2 = 0;
% paths(1).pparam3 = 0;
% paths(1).pparam4 = 0;
% paths(1).pparam5 = 80;
% paths(1).pparam6 = 1;
% paths(1).pparam7 = 0*pi/180;
% paths(1).pparam8 = -90*pi/180;
% paths(1).pparam9 = 360*2*pi/180;
% paths(2).pparam1 = 1;
% paths(2).pparam2 = 0;
% paths(2).pparam3 = 0;
% paths(2).pparam4 = 0;
% paths(2).pparam5 = 80;
% paths(2).pparam6 = 1;
% paths(2).pparam7 = 0*pi/180;
% paths(2).pparam8 = -90*pi/180;
% paths(2).pparam9 = 360*2*pi/180;


% paths(1).pparam1 = 0;
% paths(1).pparam2 = 0;
% paths(1).pparam3 = -50;
% paths(1).pparam4 = 0;
% paths(1).pparam5 = 200;
% paths(1).pparam6 = -50;
% paths(1).pparam7 = 0;
% paths(1).pparam8 = 0;
% paths(1).pparam9 = 0;

paths(1).pparam1 = 1;
paths(1).pparam2 = 0;
paths(1).pparam3 = 0;
paths(1).pparam4 = 0;
paths(1).pparam5 = 50;
paths(1).pparam6 = 1;
paths(1).pparam7 = -5*pi/180;
paths(1).pparam8 = -90*pi/180;
paths(1).pparam9 = 360*pi/180;%270*pi/180;

% paths(2).pparam1 = 1;
% paths(2).pparam2 = 0;
% paths(2).pparam3 = 0;
% paths(2).pparam4 = 28;
% paths(2).pparam5 = 50;
% paths(2).pparam6 = 1;
% paths(2).pparam7 = 5*pi/180;
% paths(2).pparam8 = -90*pi/180;
% paths(2).pparam9 = 360*2*pi/180;%270*pi/180;

paths(2).pparam1 = 1;
paths(2).pparam2 = 0;
paths(2).pparam3 = 0;
paths(2).pparam4 = 28;
paths(2).pparam5 = 50;
paths(2).pparam6 = 1;
paths(2).pparam7 = 8*pi/180;
paths(2).pparam8 = -90*pi/180;
paths(2).pparam9 = 360*4*pi/180;%270*pi/180;

% lab = 100;
% paths(2).pparam1 = 0;
% paths(2).pparam2 = paths(1).pparam2 + paths(1).pparam5 * cos(paths(1).pparam8 + paths(1).pparam6 * paths(1).pparam9);
% paths(2).pparam3 = paths(1).pparam3 + paths(1).pparam5 * sin(paths(1).pparam8 + paths(1).pparam6 * paths(1).pparam9);
% paths(2).pparam4 = paths(1).pparam4 + paths(1).pparam5 * -tan(paths(1).pparam7) * paths(1).pparam9;
% paths(2).pparam5 = paths(2).pparam2 + lab * cos(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2));
% paths(2).pparam6 = paths(2).pparam3 + lab * sin(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2));
% paths(2).pparam7 = paths(2).pparam4 + 0;
% paths(2).pparam8 = 0;
% paths(2).pparam9 = 0;


% % paths(2).pparam1 = 0;
% % paths(2).pparam2 = 200;
% % paths(2).pparam3 = -50;
% % paths(2).pparam4 = 0;
% % paths(2).pparam5 = 200;
% % paths(2).pparam6 = 200;
% % paths(2).pparam7 = 20;
% % paths(2).pparam8 = 0;
% % paths(2).pparam9 = 0;

% paths(2).pparam1 = 1;
% paths(2).pparam2 = 0;
% paths(2).pparam3 = 0;
% paths(2).pparam4 = 0;
% paths(2).pparam5 = 60;
% paths(2).pparam6 = 1;
% paths(2).pparam7 = -5*pi/180;
% paths(2).pparam8 = -90*pi/180;
% paths(2).pparam9 = 900*pi/180;
% 
% paths(3).pparam1 = 0;
% paths(3).pparam2 = 150;
% paths(3).pparam3 = 0;
% paths(3).pparam4 = 0;
% paths(3).pparam5 = 150;
% paths(3).pparam6 = -200;
% paths(3).pparam7 = 0;
% paths(3).pparam8 = 0;
% paths(3).pparam9 = 0;

% /////////////////////////////////////////////////////////////////////////

% % % paths(1).pparam1 = 1;
% % % paths(1).pparam2 = 0;
% % % paths(1).pparam3 = 60;
% % % paths(1).pparam4 = 0;
% % % paths(1).pparam5 = 40;
% % % paths(1).pparam6 = 1;
% % % paths(1).pparam7 = 10*pi/180;
% % % paths(1).pparam8 = -90*pi/180;
% % % paths(1).pparam9 = 900*pi/180;%270*pi/180;

% lab = 30;
% paths(2).pparam1 = 0;
% paths(2).pparam2 = paths(1).pparam2 + paths(1).pparam5 * cos(paths(1).pparam8 + paths(1).pparam6 * paths(1).pparam9);
% paths(2).pparam3 = paths(1).pparam3 + paths(1).pparam5 * sin(paths(1).pparam8 + paths(1).pparam6 * paths(1).pparam9);
% paths(2).pparam4 = paths(1).pparam4 + paths(1).pparam5 * -tan(paths(1).pparam7) * paths(1).pparam9;
% paths(2).pparam5 = paths(2).pparam2 + lab * cos(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2));
% paths(2).pparam6 = paths(2).pparam3 + lab * sin(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2));
% paths(2).pparam7 = paths(2).pparam4 + 0;
% paths(2).pparam8 = 0;
% paths(2).pparam9 = 0;
% 
% paths(3).pparam1 = 1;
% paths(3).pparam5 = 50;
% paths(3).pparam6 = -1;
% paths(3).pparam7 = -5*pi/180;
% paths(3).pparam2 = paths(2).pparam5 + paths(3).pparam5 * cos(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2) + paths(3).pparam6 * pi/2);
% paths(3).pparam3 = paths(2).pparam6 + paths(3).pparam5 * sin(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2) + paths(3).pparam6 * pi/2);
% paths(3).pparam4 = paths(2).pparam7 + paths(3).pparam5 * 0;
% paths(3).pparam8 = mod(paths(1).pparam8 + paths(1).pparam6 * (paths(1).pparam9 + pi/2) - paths(3).pparam6 * pi/2,2*pi);
% paths(3).pparam9 = 340*pi/180;
% 
% lab = 10;
% paths(4).pparam1 = 0;
% paths(4).pparam2 = paths(3).pparam2 + paths(3).pparam5 * cos(paths(3).pparam8 + paths(3).pparam6 * paths(3).pparam9);
% paths(4).pparam3 = paths(3).pparam3 + paths(3).pparam5 * sin(paths(3).pparam8 + paths(3).pparam6 * paths(3).pparam9);
% paths(4).pparam4 = paths(3).pparam4 + paths(3).pparam5 * -tan(paths(3).pparam7) * paths(3).pparam9;
% paths(4).pparam5 = paths(4).pparam2 + lab * cos(paths(3).pparam8 + paths(3).pparam6 * (paths(3).pparam9 + pi/2));
% paths(4).pparam6 = paths(4).pparam3 + lab * sin(paths(3).pparam8 + paths(3).pparam6 * (paths(3).pparam9 + pi/2));
% paths(4).pparam7 = paths(4).pparam4 + lab * -sin(paths(3).pparam7);
% paths(4).pparam8 = 0;
% paths(4).pparam9 = 0;
%  
% paths(5).pparam1 = 1;
% paths(5).pparam5 = 40;
% paths(5).pparam6 = 1;
% paths(5).pparam7 = 0*pi/180;
% paths(5).pparam8 = mod(paths(3).pparam8 + paths(3).pparam6 * paths(3).pparam9 + pi/2 - paths(3).pparam6 * paths(5).pparam6 * pi/2,2*pi);
% paths(5).pparam9 = 30*pi/180;
% paths(5).pparam2 = paths(4).pparam5 + paths(5).pparam5 * cos(paths(3).pparam8 + paths(3).pparam6 * (paths(3).pparam9 + pi/2) + paths(5).pparam6 * pi/2);
% paths(5).pparam3 = paths(4).pparam6 + paths(5).pparam5 * sin(paths(3).pparam8 + paths(3).pparam6 * (paths(3).pparam9 + pi/2) + paths(5).pparam6 * pi/2);
% paths(5).pparam4 = paths(4).pparam7 + paths(5).pparam5 * 0;
%  
% lab = 300;
% paths(6).pparam1 = 0;
% paths(6).pparam2 = paths(5).pparam2 + paths(5).pparam5 * cos(paths(5).pparam8 + paths(5).pparam6 * paths(5).pparam9);
% paths(6).pparam3 = paths(5).pparam3 + paths(5).pparam5 * sin(paths(5).pparam8 + paths(5).pparam6 * paths(5).pparam9);
% paths(6).pparam4 = paths(5).pparam4 + paths(5).pparam5 * -tan(paths(5).pparam7) * paths(5).pparam9;
% paths(6).pparam5 = paths(6).pparam2 + lab * cos(paths(5).pparam8 + paths(5).pparam6 * (paths(5).pparam9 + pi/2));
% paths(6).pparam6 = paths(6).pparam3 + lab * sin(paths(5).pparam8 + paths(5).pparam6 * (paths(5).pparam9 + pi/2));
% paths(6).pparam7 = paths(6).pparam4 + 0;
% paths(6).pparam8 = 0;
% paths(6).pparam9 = 0;
% 
% paths(7).type = 2;
% 
% figure('color','w','name','3D position/height');
% 
% subplot(5,1,1:4); hold on; grid on;
% 
% ltset=1000;
% for k = 1:length(paths)-1
% 
%     if paths(k).type == 0
%         plot3([paths(k).aa(2) paths(k).bb(2)], ...
%             [paths(k).aa(1) paths(k).bb(1)], ...
%             -[paths(k).aa(3) paths(k).bb(3)],'--m','linewidth',2);
%     elseif paths(k).type == 1
%         tset = linspace(paths(k).pparam8, ...
%             paths(k).pparam8 + paths(k).pparam9 * paths(k).pparam6, ...
%             ltset)';
%         r = repmat(paths(k).cc,ltset,1) + ...
%             [paths(k).pparam5 * cos(tset), paths(k).pparam5 * sin(tset), ...
%             -paths(k).pparam6 * (tset-tset(1)) * paths(k).pparam5 * tan(paths(k).pparam7)];
%         plot3(r(:,2),r(:,1),-r(:,3),'--m','linewidth',2)
%     end
%     
% end
