% bi linearly interpolate

clear;
clc;

[X_gt,Y_gt,Z_gt] = peaks(701);

dis = 0.2;
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
        
        idx_terr = lookup_terrain_idx(pos_y, pos_x, pos_y_origin, pos_x_origin,...
            dis, len_x_dis, len_y_dis, 0);
        h_terr(i,j) = z_dis_array(idx_terr+1);
        
    end
end

% bi-linear interpolation
for i=1:length(X_gt(1,:))
    for j=1:length(Y_gt(:,1))
        
        pos_x = X_gt(1,i);
        pos_y = Y_gt(j,1);
        
        idx_terr = lookup_terrain_idx(pos_y, pos_x, pos_y_origin, pos_x_origin,...
            dis, len_x_dis, len_y_dis, 1);
        h_terr(i,j) = z_dis_array(idx_terr+1);
        
    end
end

figure('color','w'); hold on; grid on; box on;
surf(X_gt(1,:),Y_gt(:,1),Z_gt,'edgecolor','none')
surf(X_gt(1,:),Y_gt(:,1),h_terr+10,'edgecolor','none')


function idx_terr = lookup_terrain_idx(pos_n, pos_e, pos_n_origin, pos_e_origin, ...
    dis, len_x_dis, len_y_dis, opt)

    ONE_DIS = 1/dis;
    LEN_IDX_N = len_y_dis;
    LEN_IDX_E = len_x_dis;

    % current block
    rel_n = pos_n - pos_n_origin;
    rel_n_normalized = rel_n * ONE_DIS;
    idx_n = floor(rel_n_normalized);
    if (idx_n < 0)
        idx_n = 0;
    elseif (idx_n >= LEN_IDX_N) 
        idx_n = LEN_IDX_N - 1;
    end
    if (rel_n_normalized-idx_n<0.5)
        neighbor_n=max(idx_n-1,0);
        neighbor_n_first = true;
    else
        neighbor_n=min(idx_n+1,LEN_IDX_N-1);
        neighbor_n_first = false;
    end
    
    rel_e = pos_e - pos_e_origin;
    rel_e_normalized = rel_e * ONE_DIS;
    idx_e = floor(rel_e_normalized);
    if (idx_e < 0) 
        idx_e = 0;
    elseif (idx_e >= LEN_IDX_E) 
        idx_e = LEN_IDX_E - 1;
    end
    if (rel_e_normalized-idx_e<0.5)
        neighbor_e=max(idx_e-1,0);
        neighbor_e_first = true;
    else
        neighbor_e=min(idx_e+1,LEN_IDX_E-1);
        neighbor_e_first = false;
    end
    
    idx_terr_P = idx_n*LEN_IDX_E + idx_e;
    
    if (neighbor_n_first && neighbor_e_first)
        Q11=[neighbor_n,neighbor_e];
        
    idx_Q = [neighbor_n*LEN_IDX_E + idx_e
    
    
    idx_terr = 
    
end

% function bilinear_interp()
% 
% 
% R1 = ((x2 – x)/(x2 – x1))*Q11 + ((x – x1)/(x2 – x1))*Q21
% 
% R2 = ((x2 – x)/(x2 – x1))*Q12 + ((x – x1)/(x2 – x1))*Q22
% 
% After the two R values are calculated, the value of P can finally be calculated by a weighted average of R1 and R2.
% 
% P = ((y2 – y)/(y2 – y1))*R1 + ((y – y1)/(y2 – y1))*R2
% 
% end;