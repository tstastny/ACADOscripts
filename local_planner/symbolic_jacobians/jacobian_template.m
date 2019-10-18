clear; clc;

% lsq objectives
y = {'v_n','v_e','v_d','v','phi','theta','sig_aoa','sig_h','sig_r', 'uT', 'phi_ref', 'theta_ref'};
len_y = length(y);

% lsq end term objectives
yN = {'v_n','v_e','v_d','v','phi','theta','sig_aoa','sig_h','sig_r'};
len_yN = length(yN);

% states
% r_n, r_e, r_d, v, gamma, xi, phi, theta, n_p
len_x = 9;

% controls
% uT, phi_ref, theta_ref
len_u = 3;

% lsq jacobian state dependencies
dydx_dep = zeros(len_y, len_x);
dydx_dep(1,:) = [0 0 0 1 1 1 0 0 0]; % v_n
dydx_dep(2,:) = [0 0 0 1 1 1 0 0 0]; % v_e
dydx_dep(3,:) = [0 0 0 1 1 1 0 0 0]; % v_d
dydx_dep(4,:) = [0 0 0 1 0 0 0 0 0]; % v
dydx_dep(5,:) = [0 0 0 0 0 0 1 0 0]; % phi
dydx_dep(6,:) = [0 0 0 0 0 0 0 1 0]; % theta
dydx_dep(7,:) = [0 0 0 0 1 0 0 1 0]; % sig_aoa
dydx_dep(8,:) = [1 1 1 0 0 1 0 0 0]; % sig_h
dydx_dep(9,:) = [1 1 1 1 1 1 0 0 0]; % sig_r
dydx_dep(10,:) = [0 0 0 0 0 0 0 0 0]; % uT
dydx_dep(11,:) = [0 0 0 0 0 0 0 0 0]; % phi_ref
dydx_dep(12,:) = [0 0 0 0 0 0 0 0 0]; % theta_ref

% jacobian control dependencies
dydu_dep = zeros(len_y, len_u);
dydu_dep(1,:) = [0 0 0]; % v_n
dydu_dep(2,:) = [0 0 0]; % v_e
dydu_dep(3,:) = [0 0 0]; % v_d
dydu_dep(4,:) = [0 0 0]; % v
dydu_dep(5,:) = [0 0 0]; % phi
dydu_dep(6,:) = [0 0 0]; % theta
dydu_dep(7,:) = [0 0 0]; % sig_aoa
dydu_dep(8,:) = [0 0 0]; % sig_h
dydu_dep(9,:) = [0 0 0]; % sig_r
dydu_dep(10,:) = [1 0 0]; % uT
dydu_dep(11,:) = [0 1 0]; % phi_ref
dydu_dep(12,:) = [0 0 1]; % theta_ref

% lsq end term jacobian state dependencies
dyNdx_dep = dydx_dep(1:len_yN, 1:len_x);

k=0;

% lsq jacobians
k=k+1; txt{k} = ['/* lsq non-zero jacobian evals */'];
for j = 1:len_y
    sumj = sum(dydx_dep(j,:));
    if sumj==1
        k=k+1; txt{k} = ['double jac_',y{j},';'];
    elseif sumj>0
        k=k+1; txt{k} = ['double jac_',y{j},'[',int2str(sumj),'];'];
    end
end
for j = 1:len_y
    sumj = sum(dydu_dep(j,:));
    if sumj==1
        k=k+1; txt{k} = ['double jac_',y{j},';'];
    elseif sumj > 0
        k=k+1; txt{k} = ['double jac_',y{j},'[',int2str(sum(dydu_dep(j,:))),'];'];
    end
end
k=k+1; txt{k} = '\n';
k=k+1; txt{k} = ['/* lsq jacobian w.r.t. states */'];
for j = 0:len_y-1
    for i = 0:len_x-1
        k=k+1;
        if dydx_dep(j+1,i+1)
            if sum(dydx_dep(j+1,:))==1
                txt{k} = ['out[',int2str(len_y+j*len_x+i),'] = jac_',y{j+1},';'];
            else
                txt{k} = ['out[',int2str(len_y+j*len_x+i),'] = jac_',y{j+1},'[',int2str(sum(dydx_dep(j+1,1:i+1))-1),'];'];
            end
        else
            txt{k} = ['out[',int2str(len_y+j*len_x+i),'] = 0.0;'];
        end
    end
end
k=k+1; txt{k} = '\n';
k=k+1; txt{k} = ['/* lsq jacobian w.r.t. controls */']; 
for j = 0:len_y-1
    for i = 0:len_u-1
        k=k+1;
        if dydu_dep(j+1,i+1)
            if sum(dydu_dep(j+1,:))==1
                txt{k} = ['out[',int2str(len_y+len_y*len_x+j*len_u+i),'] = jac_',y{j+1},';'];
            else
                txt{k} = ['out[',int2str(len_y+len_y*len_x+j*len_u+i),'] = jac_',y{j+1},'[',int2str(sum(dydu_dep(j+1,1:i+1))-1),'];'];
            end
        else
            txt{k} = ['out[',int2str(len_y+len_y*len_x+j*len_u+i),'] = 0.0;'];
        end
    end
end
k=k+1; txt{k} = '\n';

% lsq end term jacobians
k=k+1; txt{k} = ['/* lsq end term non-zero jacobian evals */'];
for j = 1:len_yN
    sumj = sum(dydx_dep(j,:));
    if sumj == 1
        k=k+1; txt{k} = ['double jac_',yN{j},';'];
    elseif sumj > 0
        k=k+1; txt{k} = ['double jac_',yN{j},'[',int2str(sum(dyNdx_dep(j,:))),'];'];
    end
end
k=k+1; txt{k} = '\n';
k=k+1; txt{k} = ['/* lsq end term jacobian w.r.t. states */'];
for j = 0:len_yN-1
    for i = 0:len_x-1
        k=k+1;
        if dyNdx_dep(j+1,i+1)
            if sum(dyNdx_dep(j+1,:))==1
                txt{k} = ['out[',int2str(len_yN+j*len_x+i),'] = jac_',yN{j+1},';'];
            else
                txt{k} = ['out[',int2str(len_yN+j*len_x+i),'] = jac_',yN{j+1},'[',int2str(sum(dyNdx_dep(j+1,1:i+1))-1),'];'];
            end
        else
            txt{k} = ['out[',int2str(len_yN+j*len_x+i),'] = 0.0;'];
        end
    end
end

% write to file
fid = fopen('jac_output_ccode.c','w');
for k = 1:length(txt)
    fprintf(fid,[char(txt{k}),'\n']);
end
fclose(fid);