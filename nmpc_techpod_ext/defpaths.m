% dubins paths list

% paths(1).type   = 0;
% paths(1).aa     = [0, 0, 0];
% paths(1).bb     =  [2000, 0, 0];



paths(1).type    = 1;
paths(1).R       = 40;
paths(1).ldir    = 1;
paths(1).gam     = 10*pi/180;
paths(1).xi0     = -90*pi/180;
paths(1).deltaxi = 270*pi/180;
paths(1).cc      = [0, 60, 0];

lab = 30;
paths(2).type    = 0;
paths(2).aa      = paths(1).cc + paths(1).R * ...
    [cos(paths(1).xi0 + paths(1).ldir * paths(1).deltaxi), ...
     sin(paths(1).xi0 + paths(1).ldir * paths(1).deltaxi), ...
     -tan(paths(1).gam) * paths(1).deltaxi];
paths(2).bb     = paths(2).aa + ...
    [lab * cos(paths(1).xi0 + paths(1).ldir * (paths(1).deltaxi + pi/2)), ...
     lab * sin(paths(1).xi0 + paths(1).ldir * (paths(1).deltaxi + pi/2)), ...
     0];

paths(3).type    = 1;
paths(3).R       = 50;
paths(3).ldir    = -1;
paths(3).gam     = -5*pi/180;
paths(3).xi0     = mod(paths(1).xi0 + paths(1).ldir * (paths(1).deltaxi + pi/2) - paths(3).ldir * pi/2,2*pi);
paths(3).deltaxi = 340*pi/180;
paths(3).cc      = paths(2).bb + paths(3).R * ...
    [cos(paths(1).xi0 + paths(1).ldir * (paths(1).deltaxi + pi/2) + paths(3).ldir * pi/2), ...
     sin(paths(1).xi0 + paths(1).ldir * (paths(1).deltaxi + pi/2) + paths(3).ldir * pi/2), ...
     0];
 
lab = 10;
paths(4).type    = 0;
paths(4).aa      = paths(3).cc + paths(3).R * ...
    [cos(paths(3).xi0 + paths(3).ldir * paths(3).deltaxi), ...
     sin(paths(3).xi0 + paths(3).ldir * paths(3).deltaxi), ...
     -tan(paths(3).gam) * paths(3).deltaxi];
paths(4).bb     = paths(4).aa + ...
    [lab * cos(paths(3).xi0 + paths(3).ldir * (paths(3).deltaxi + pi/2)), ...
     lab * sin(paths(3).xi0 + paths(3).ldir * (paths(3).deltaxi + pi/2)), ...
     lab * -sin(paths(3).gam)];
 
paths(5).type    = 1;
paths(5).R       = 40;
paths(5).ldir    = 1;
paths(5).gam     = 0*pi/180;
paths(5).xi0     = mod(paths(3).xi0 + paths(3).ldir * paths(3).deltaxi + pi/2 - paths(3).ldir * paths(5).ldir * pi/2,2*pi);
paths(5).deltaxi = 30*pi/180;
paths(5).cc      = paths(4).bb + paths(5).R * ...
    [cos(paths(3).xi0 + paths(3).ldir * (paths(3).deltaxi + pi/2) + paths(5).ldir * pi/2), ...
     sin(paths(3).xi0 + paths(3).ldir * (paths(3).deltaxi + pi/2) + paths(5).ldir * pi/2), ...
     0];
 
lab = 300;
paths(6).type    = 0;
paths(6).aa      = paths(5).cc + paths(5).R * ...
    [cos(paths(5).xi0 + paths(5).ldir * paths(5).deltaxi), ...
     sin(paths(5).xi0 + paths(5).ldir * paths(5).deltaxi), ...
     -tan(paths(5).gam) * paths(5).deltaxi];
paths(6).bb     = paths(6).aa + ...
    [lab * cos(paths(5).xi0 + paths(5).ldir * (paths(5).deltaxi + pi/2)), ...
     lab * sin(paths(5).xi0 + paths(5).ldir * (paths(5).deltaxi + pi/2)), ...
     0];
 
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
%         tset = linspace(paths(k).xi0, ...
%             paths(k).xi0 + paths(k).deltaxi * paths(k).ldir, ...
%             ltset)';
%         r = repmat(paths(k).cc,ltset,1) + ...
%             [paths(k).R * cos(tset), paths(k).R * sin(tset), ...
%             -paths(k).ldir * (tset-tset(1)) * paths(k).R * tan(paths(k).gam)];
%         plot3(r(:,2),r(:,1),-r(:,3),'--m','linewidth',2)
%     end
%     
% end
