% bi linearly interpolate

clear;
clc;

[X_gt,Y_gt,Z_gt] = peaks(701);

dis = 0.5;
x_dis = X_gt(1,1):dis:X_gt(1,end);
y_dis = Y_gt(1,1):dis:Y_gt(end,1);
len_x_dis = length(x_dis);
len_y_dis = length(y_dis);
for i=1:len_x_dis
    idx_x_dis = find(X_gt(1,:)>=x_dis(i),1,'first');
    for j=1:len_y_dis
        idx_y_dis = find(Y_gt(:,1)>=y_dis(j),1,'first');
        z_dis(i,j) = Z_gt(idx_x_dis,idx_y_dis);
    end
end
z_dis_array = reshape(z_dis,1,len_x_dis*len_y_dis);

pos_x_origin=X_gt(1,1)-dis/2;
pos_y_origin=Y_gt(1,1)-dis/2;

% nearest neighbor lookup
for i=1:length(X_gt(1,:))
    for j=1:length(Y_gt(:,1))
        
        pos_x = X_gt(1,i);
        pos_y = Y_gt(j,1);
        
        % nearest neighborhood lookup
        idx_ = lookup_terrain_idx(pos_y, pos_x, pos_y_origin, pos_x_origin,...
            dis, len_x_dis, len_y_dis);
        h_terr(i,j) = z_dis_array(idx_(1)+1);
        
    end
end

% bi-linear interpolation
for i=1:length(X_gt(1,:))
    for j=1:length(Y_gt(:,1))
        
        pos_x = X_gt(1,i);
        pos_y = Y_gt(j,1);
        
        idx_ = lookup_terrain_idx(pos_y, pos_x, pos_y_origin, pos_x_origin,...
            dis, len_x_dis, len_y_dis);
        
        % bi-linear interp
        h1 = z_dis_array(idx_(1+1)+1);
        h2 = z_dis_array(idx_(2+1)+1);
        h3 = z_dis_array(idx_(3+1)+1);
        h4 = z_dis_array(idx_(4+1)+1);
        h12 = (1-idx_(5+1))*h1 + idx_(5+1)*h2;
        h34 = (1-idx_(5+1))*h3 + idx_(5+1)*h4;
        h_terr_bilin(i,j) = (1-idx_(6+1))*h12 + idx_(6+1)*h34;
        
    end
end

figure('color','w'); hold on; grid on; box on;
surf(X_gt(1,:),Y_gt(:,1),Z_gt,'edgecolor','none')
surf(X_gt(1,:),Y_gt(:,1),h_terr+15,'edgecolor','none')
surf(X_gt(1,:),Y_gt(:,1),h_terr_bilin+15*2,'edgecolor','none')


function idx_ = lookup_terrain_idx(pos_n, pos_e, pos_n_origin, pos_e_origin, ...
    dis, len_x_dis, len_y_dis)
    
    ONE_DIS = 1/dis;
    LEN_IDX_N = len_y_dis;
    LEN_IDX_E = len_x_dis;
    LEN_IDX_N_1 = LEN_IDX_N - 1;
    LEN_IDX_E_1 = LEN_IDX_E - 1;

    rel_n = pos_n - pos_n_origin;
    rel_n_bar = rel_n * ONE_DIS;
    idx_n = floor(rel_n_bar); % int idx_n = rel_n_bar;
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n > LEN_IDX_N_1) 
        idx_n = LEN_IDX_N_1;
    end
    delta_n = rel_n_bar-idx_n;
    if (delta_n<0.5) % int up = (rel_n_bar-idx_n<0.5) ? 0 : 1;
        down = -1;
        dh_n = 0.5 + delta_n;
    else
        down = 0;
        dh_n = delta_n - 0.5;
    end
    
    rel_e = pos_e - pos_e_origin;
    rel_e_bar = rel_e * ONE_DIS;
    idx_e = floor(rel_e_bar); % int idx_e = rel_e_bar;
    if (idx_e < 0) 
        idx_e = 0;
    elseif (idx_e > LEN_IDX_E_1) 
        idx_e = LEN_IDX_E_1;
    end
    delta_e = rel_e_bar-idx_e;
    if (delta_e<0.5) % int left = (rel_e_bar-idx_n<0.5) ? -1 : 0;
        left = -1;
        dh_e = 0.5 + delta_e;
    else
        left = 0;
        dh_e = delta_e - 0.5;
    end
    
    % current block
    idx_terr = idx_n*LEN_IDX_E + idx_e;
    
    % neighbor origin (down,left)
    Q1_n = idx_n + down;
    Q1_e = idx_e + left;
    
    % cap neighbors (north)
    if (Q1_n >= LEN_IDX_N_1)
        Q_n = [LEN_IDX_N_1 LEN_IDX_N_1 LEN_IDX_N_1 LEN_IDX_N_1];
    else
        Q_n = [Q1_n (Q1_n + 1) Q1_n (Q1_n + 1)];
    end
    % cap neighbors (east)
    if (Q1_e >= LEN_IDX_N_1)
        Q_e = [LEN_IDX_N_1 LEN_IDX_N_1 LEN_IDX_N_1 LEN_IDX_N_1];
    else
        Q_e = [Q1_e Q1_e (Q1_e + 1) (Q1_e + 1)];
    end
    
    Q = [Q_n(1)*LEN_IDX_E + Q_e(1), ...
        Q_n(2)*LEN_IDX_E + Q_e(2), ...
        Q_n(3)*LEN_IDX_E + Q_e(3), ...
        Q_n(4)*LEN_IDX_E + Q_e(4)];
    
    idx_ = [idx_terr, Q, dh_n, dh_e];
end
