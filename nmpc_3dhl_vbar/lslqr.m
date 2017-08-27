function [K,Js] = lslqr(A,B,C,Q,R,k0,iter,tol,alpha)
% =========================================================================
% LIMITED STATE OPTIMAL OUTPUT FEEDBACK CONTROLLER DESIGN
% Continous System Dynamics
% By: Thomas Stastny (Last update: Spring 2013)
% =========================================================================
% This function uses a standard gradient descent based algorithm to search
% for an optimal set of gains K that minimize the output feedback problem
% for continuous states. Global optimums and closed loop stability are NOT
% guaranteed as in the standard LQR problem.
%
% Inputs:
% A,B   = Open loop system dynamics
% C     = Observation matrix
% Q,R   = State and control, respectively, weighting matrices
% k0    = Initial guess for closed loop gains (full state feedback)
% iter  = Maximum number of algorithm iterations
% tol   = Algorithm convergence tolerance
% alpha = Algorithm step size
%
% Outputs:
% K = Optimal closed loop gain solution
% Js = Cost history
%
% Non-MATLAB-standard function calls necessary for algorithm:
% - reshapek0.m
% -------------------------------------------------------------------------
%
% TIPS FOR COST MINIMIZATION PARAMETER SELECTION
% ----------------------------------------------
% The implemented minimization algorithm is VERY basic. It may need some
% help in finding optimal gain matrices.
%
% If the algorithm stops due to an unstable gain matrix...
%   1)  Reduce alpha, try again
%   2)  Check cost values and consider relaxing convergence criterion
%
% If the algorithm does not converge within the maximum iterations...
%   1)  Increase alpha (until just before the algorithm starts producing
%       unstable closed-loop systems)
%   2)  Increase maximum number of iterations
%   3)  Check cost values and consider relaxing convergence criterion
%
% If the algorithm converges within very few iterations...
%   1)  Consider increasing alpha to avoid a local minimum
%   2)  Consider tightening the tolerance
% -------------------------------------------------------------------------

ns = size(A,1);     % Number of states
nc = size(B,2);     % Number of controls
nsfb = size(C,1);   % Number of states fed back

K = zeros(nc,ns);

if nargin <= 8
    alpha = 0.1;
end
if nargin <= 7
    tol = 10e-4;
end
if nargin <= 6
    iter = 1000;
end
if nargin <= 5
% If no initial guess is provided, compute using full state feedback LQR
    k0 = lqr(A,B,Q,R);
end

X = eye(max(size(A)));
J0 = 1e+6;

% Check for controllability
OCM = zeros(nsfb,ns*nc);
for i = 0:ns-1
    OCM(:,i*nc+1:i*nc+nc) = C*A^i*B;
end
if rank(OCM) ~= nsfb
    error('Output controllability matrix is not full rank. Script stopped.')
end

% Reshape k0
k = reshapek0(A,C,k0);

Js = zeros(iter,2);     % cost tracking

% Search for optimal gain K
for i = 1:iter  % START SEARCH FOR LOOP -----------------------------------
    
    % Compute closed loop system matrix
    Ac = A - B*k*C;
    e = eig(Ac);
    
    % Check for unstable modes
    for ii = 1:size(e)
        if real(e(ii)) >= 0
            disp(['Bad k matrix on iteration ',int2str(i),'.'])
            disp('Algorithm stopped.')
            disp(' ')
            disp('Gain matrix:')
            k
            disp('Closed loop eigen values:')
            e
            Js = Js(1:i,:);
            return
        end
    end
    
    % Calculate Riccati solutions
    Q1 = C'*k'*R*k*C + Q;
    P = lyap(Ac',Q1);
    S = lyap(Ac,X);
    
    % Calculate cost
    J = 0.5*trace(P*X);
    err = abs(J-J0);
    if err <= tol
        disp(['Algorithm converged on iteration ',int2str(i),'.'])
        disp(' ')
        disp('Gain matrix:')
        K = k
        disp('Closed loop eigen values:')
        e
        Js = [Js(1:i-1,:); J err];
        return
    end
    J0 = J;
    
    % Update k
    deltak = R\B'*P*S*C'/(C*S*C') - k;
    k = k + alpha*deltak;
    
    Js(i,:) = [J err];
    
end

K = k;
Js = Js(1:i,:);
disp(['Algorithm did not converge in ',int2str(iter),' iterations.'])
return    
    
    