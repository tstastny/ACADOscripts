% experiment info
az = [120:-2:0,358:-2:122];
el = 50;

% Movie
nFrames = length(az);
vidObj = VideoWriter('3d_spin.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 20;
open(vidObj);

for kk = 1:nFrames

    figure('color','w','name','3D position/height');

    subplot(5,1,1:4); hold on; grid on;

    ltset=1000;
    for k = 1:length(paths)

        if paths(k).type == 0
            plot3([paths(k).aa(2) paths(k).bb(2)], ...
                [paths(k).aa(1) paths(k).bb(1)], ...
                -[paths(k).aa(3) paths(k).bb(3)],'--m','linewidth',2);
        elseif paths(k).type == 1
            tset = linspace(paths(k).xi0, ...
                paths(k).xi0 + paths(k).deltaxi * paths(k).ldir, ...
                ltset)';
            r = repmat(paths(k).cc,ltset,1) + ...
                [paths(k).R * cos(tset), paths(k).R * sin(tset), ...
                -paths(k).ldir * (tset-tset(1)) * paths(k).R * tan(paths(k).gam)];
            plot3(r(:,2),r(:,1),-r(:,3),'--m','linewidth',2)
        end

    end
    plot3(X_rec(:,11),X_rec(:,10),-X_rec(:,12));
    xlabel('e [m]'); ylabel('n [m]'); zlabel('h [m]');
    view([az(kk),el])
    subplot(5,1,5); hold on; grid on;
    plot(time',-X_rec(:,12)); ylabel('h [m]');
    xlabel('t [s]')

    % Record Frame
    writeVideo(vidObj, getframe(gcf));
    close(gcf)

    clc
    disp(['Frame ',int2str(kk),' recorded'])
end

clc
disp('Done.')
close(vidObj);