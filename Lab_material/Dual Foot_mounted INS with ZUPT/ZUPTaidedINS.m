function [x_h cov] = ZUPTaidedINS(u, u_c, zupt, heading)

% Global struct holding the simulation settings
global simdata

% Extract the ZUPT vectors of Right and Left foot INS.
zupt_r = zupt(1,:);
zupt_l = zupt(2,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                       Initialize the data fusion                      %%       
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Get  the length of the IMU data vector
N = length(u);

% Initialize the filter state covariance matrix P, the processes noise
% covariance matrix Q of Right and Left foot INS.
[P_r Q_r] = init_filter;          % Subfunction located further down in the file.
[P_l Q_l] = init_filter;          % Subfunction located further down in the file.
P_h = (1e-4)*eye(9);   Q_h=(1e-14)*eye(9);

% The observation matrix H of velocity
H_v = [zeros(3) eye(3) zeros(3)];
% The observation matrix H of position
H_p = [eye(3) zeros(3,6)];
% The pseudo measurement noise covariance matrix R of velocity
R_v=diag(simdata.sigma_vel.^2);
% The pseudo measurement noise covariance matrix R of position
R_p=diag(simdata.sigma_pos.^2);

% Zero velocity pseudo-measurement vector
vel_pseudo_measurement = zeros(3,1);

% Allocate vecors
[x_h_r cov_r] = init_vec(N, P_r);     % Subfunction located further down in the file.
[x_h_l cov_l] = init_vec(N, P_l);     % Subfunction located further down in the file.

% Initialize the navigation state vector x_h, and the quaternion vector
% quat of Right and Left foot INS.
[x_h_r(1:9,1) quat_r] = init_Nav_eq(u(1:6,:));  % Subfunction located further down in the file.
[x_h_l(1:9,1) quat_l] = init_Nav_eq(u(7:12,:)); 
%[x_h_l(1:9,1) quat_l] = init_Nav_eq(u_c(1:6,:));% Subfunction located further down in the file.

last_update = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                        Run the filter algorithm                       %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 2:N
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                             Time Update                             %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Compensate the IMU measurements with the current estimates of the 
    % sensor errors for Right and Left foot. 
    u_r = comp_imu_errors(u(1:6,k), x_h_r(:,k-1)); % Subfunction located further down in the file.
    %u_l = comp_imu_errors(u_c(1:6,k), x_h_l(:,k-1)); % Subfunction located further down in the file.   
    u_l = comp_imu_errors(u(7:12,k), x_h_l(:,k-1)); % Subfunction located further down in the file. 
  
    % Update the navigation equations of Right and Left foot INS.   
    [x_h_r(:,k) quat_r]=Navigation_equations(x_h_r(:,k-1),u_r,quat_r, heading(k)); % Subfunction located further down in the file.
    [x_h_l(:,k) quat_l]=Navigation_equations(x_h_l(:,k-1),u_l,quat_l, heading(k)); % Subfunction located further down in the file.
  
    % Update state transition matrix of Right and Left foot INS.
    [F_r G_r] = state_matrix(quat_r, u_r); % Subfunction located further down in the file.
    [F_l G_l] = state_matrix(quat_l, u_l); % Subfunction located further down in the file.
    
    % Update the filter state covariance matrix P  for Left and Right foot.
    P_r = F_r*P_r*F_r' + G_r*Q_r*G_r';
    P_l = F_l*P_l*F_l' + G_l*Q_l*G_l';
    
    % Make sure the filter state covariance matrix is symmetric. 
    P_r = (P_r + P_r')/2;
    P_l = (P_l + P_l')/2;
    
    % Store the diagonal of the state covariance matrix P.
    cov_r(:,k) = diag(P_r);
    cov_l(:,k) = diag(P_l);   
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                        Zero-velocity update                         %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if zupt_r(k) == true                
        [x_h_r(1:9,k) P_r quat_r] = MeasurementUpdate(vel_pseudo_measurement, H_v, x_h_r(1:9,k), P_r, quat_r, R_v);
        cov_r(:,k) = diag(P_r);
         
    end
    if zupt_l(k) == true
        [x_h_l(1:9,k) P_l quat_l] = MeasurementUpdate(vel_pseudo_measurement, H_v, x_h_l(1:9,k), P_l, quat_l, R_v);
        cov_l(:,k) = diag(P_l);
        
    end  
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                           Centroid Method                           %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Distance between the position estimates of right and left foot INS
    d_h = norm(x_h_r(1:3,k)-x_h_l(1:3,k));
    if strcmp(simdata.Centroid_Method,'on') && d_h>simdata.range_constraint && (k-last_update)>simdata.min_rud_sep
        
        last_update = k;
        
        % Define the centroid
        p_0=(x_h_r(1:3,k)+x_h_l(1:3,k))/2;
        if strcmp(simdata.WeightedCentroid,'on')
            if cov_r(1,k)~=0 && cov_r(2,k)~=0 && cov_r(3,k)~=0 && cov_l(1,k)~=0 && cov_l(2,k)~=0 && cov_l(3,k)~=0
                p_0(1)=(cov_r(1,k)*x_h_l(1,k) + cov_l(1,k)*x_h_r(1,k))/(cov_r(1,k)+cov_l(1,k));
                p_0(2)=(cov_r(2,k)*x_h_l(2,k) + cov_l(2,k)*x_h_r(2,k))/(cov_r(2,k)+cov_l(2,k));
                p_0(3)=(cov_r(3,k)*x_h_l(3,k) + cov_l(3,k)*x_h_r(3,k))/(cov_r(3,k)+cov_l(3,k));
            end
        end
        
        % Calculate the range constrained Right and Left foot position
        [pos_pseudo_measurement_r(1:3) pos_pseudo_measurement_l(1:3)] = Centroid_Method(x_h_r(1:3,k),x_h_l(1:3,k),p_0);

        % Correct the Right foot navigation states using position pseudo-measurement
        [x_h_r(1:9,k) P_r quat_r]=MeasurementUpdate(pos_pseudo_measurement_r', H_p, x_h_r(1:9,k), P_r, quat_r, R_p);
        cov_r(:,k)=diag(P_r);
        
        % Correct the Left foot navigation states using position pseudo-measurement 
        [x_h_l(1:9,k) P_l quat_l]=MeasurementUpdate(pos_pseudo_measurement_l', H_p, x_h_l(1:9,k),P_l,quat_l, R_p);
        cov_l(:,k)=diag(P_l);
    end 
    
    %[x_h_r(1:9,k), P_h, quat_r] = MeasurementUpdate_Heading(heading(k), x_h_r(1:9,k), P_h, quat_r);
    %[x_h_l(1:9,k), P_h, quat_l] = MeasurementUpdate_Heading(heading(k), x_h_l(1:9,k), P_h, quat_l);
   

end

cov=[cov_r ; cov_l];
x_h=[x_h_r ; x_h_l];

end

function [x_h cov]=init_vec(N,P)

global simdata


% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    cov=zeros(9+6+6,N);
    x_h=zeros(9+6+6,N);
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    cov=zeros(9+6,N);
    x_h=zeros(9+6,N);
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    cov=zeros(9+6,N);
    x_h=zeros(9+6,N);
else
    % Only the standard errors included
    cov=zeros(9,N);
    x_h=zeros(9,N);
end

cov(:,1)=diag(P);
end

function [x quat]=init_Nav_eq(u)

global simdata;

% Under the assumption that the system is stationary during the first 20
% samples, the initial roll and pitch is calculate from the 20 first
% accelerometer readings.
f_u=mean(u(1,1:20));
f_v=mean(u(2,1:20));
f_w=mean(u(3,1:20));

roll=atan2(-f_v,-f_w);
pitch=atan2(f_u,sqrt(f_v^2+f_w^2));

% Set the attitude vector
attitude=[roll pitch simdata.init_heading]';

% Calculate quaternion corresponing to the initial attitude
Rb2t=Rt2b(attitude)';
quat=dcm2q(Rb2t);

% Set the initial state vector
x=zeros(9,1);
x(1:3,1)=simdata.init_pos;
x(7:9,1)=attitude;
end

function [P Q]=init_filter

global simdata;

% Check which errors that are included in the state space model and
% allocate P,Q, and H matrices with the right size.

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on')) % Both scale and bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    P(16:18,16:18)=diag(simdata.sigma_initial_acc_scale.^2);
    P(19:21,19:21)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    
    
    Q=zeros(12);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);

    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off') % Scale errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_scale.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_scale.^2);
    
    % Process noise covariance matrix
    Q=zeros(6);

    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on') % Bias errors included
    
    
    
    %  Initial state covariance matrix
    P=zeros(9+6);
    P(10:12,10:12)=diag(simdata.sigma_initial_acc_bias.^2);
    P(13:15,13:15)=diag(simdata.sigma_initial_gyro_bias.^2);
    
    % Process noise covariance matrix
    Q=zeros(12);
    Q(7:9,7:9)=diag(simdata.acc_bias_driving_noise.^2);
    Q(10:12,10:12)=diag(simdata.gyro_bias_driving_noise.^2);   

    
else % Only the standard errors included
    
    %  Initial state covariance matrix
    P=zeros(9);
    
    % Process noise covariance matrix
    Q=zeros(6);
    
end

% General values for the initial covariance matrix P
P(1:3,1:3)=diag(simdata.sigma_initial_pos.^2);
P(4:6,4:6)=diag(simdata.sigma_initial_vel.^2);
P(7:9,7:9)=diag(simdata.sigma_initial_att.^2);

% General values for the process noise covariance matrix Q
Q(1:3,1:3)=diag(simdata.sigma_acc.^2);
Q(4:6,4:6)=diag(simdata.sigma_gyro.^2);

end

function [y,q]=Navigation_equations(x,u,q,heading)

global simdata;

% Allocate memmory for the output vector
y=zeros(size(x));

% Get sampling period of the system
Ts=simdata.Ts;

%*************************************************************************%
% Update the quaternion vector "q"  given the angular rate measurements.
%*************************************************************************%

w_tb=u(4:6);

P=w_tb(1)*Ts;
Q=w_tb(2)*Ts;
R=w_tb(3)*Ts;

OMEGA=zeros(4);
OMEGA(1,1:4)=0.5*[0 R -Q P];
OMEGA(2,1:4)=0.5*[-R 0 P Q];
OMEGA(3,1:4)=0.5*[Q -P 0 R];
OMEGA(4,1:4)=0.5*[-P -Q -R 0];

v=norm(w_tb)*Ts;

if v~=0
    q=(cos(v/2)*eye(4)+2/v*sin(v/2)*OMEGA )*q;
    q=q./norm(q);
end

%*************************************************************************%
% Use the update quaternion to get attitude of the navigation system in
% terms of Euler angles.
%*************************************************************************%

% Get the roll, pitch and yaw
Rb2t=q2dcm(q);
% roll
y(7)=atan2(Rb2t(3,2),Rb2t(3,3));
%y(7)=heading;

% pitch
y(8)=-atan(Rb2t(3,1)/sqrt(1-Rb2t(3,1)^2));

%yaw
y(9)=atan2(Rb2t(2,1),Rb2t(1,1));
%y(9)=heading;


%*************************************************************************%
% Update position and velocity states using the measured specific force,
% and the newly calculated attitude.
%*************************************************************************%

% Gravity vector
g_t=[0 0 simdata.g]';

% Transform the specificforce vector into navigation coordinate frame.
f_t=q2dcm(q)*u(1:3);
%f_t=RR*u(1:3);

% Subtract (add) the gravity, to obtain accelerations in navigation
% coordinat system.
acc_t=f_t+g_t;

% State space model matrices
A=eye(6);
A(1,4)=Ts;
A(2,5)=Ts;
A(3,6)=Ts;

B=[60*(Ts^2)/2*eye(3);Ts*eye(3)];

% Update the position and velocity estimates.
y(1:6)=A*x(1:6)+B*acc_t;

end

function [F G]=state_matrix(q,u)

global simdata

% Convert quaternion to a rotation matrix
Rb2t=q2dcm(q);

% Transform measured force to force in
% the navigation coordinate system.
f_t=Rb2t*u(1:3);

% Create a ske symmetric matrix of the specific fore vector
St=[0 -f_t(3) f_t(2); f_t(3) 0 -f_t(1); -f_t(2) f_t(1) 0];

% Zero matrix
O=zeros(3);

% Identity matrix
I=eye(3);

% Diagonal matrices with the specific fore and angular rate along the
% diagonals.
Da=diag(u(1:3));
Dg=diag(u(4:6));

% Correlation constant for accelerometer and gyro biases
B1=-1/simdata.acc_bias_instability_time_constant_filter*eye(3);
B2=-1/simdata.gyro_bias_instability_time_constant_filter*eye(3);

% Check which errors that are included in the state space model
if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    % Both scale and bias errors included
    Fc=[O I O   O     O     O         O    ;
        O O St Rb2t   O    Rb2t*Da    O    ;
        O O O   O   -Rb2t   O     -Rb2t*Dg ;
        O O O   B1    O     O         O    ;
        O O O   O     B2    O         O    ;
        O O O   O     O     O         O    ;
        O O O   O     O     O         O   ];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I; O O O O; O O O O];
    
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    % Scale errors included
    Fc=[O I O       O       O    ;
        O O St  Rb2t*Da     O    ;
        O O O       O   -Rb2t*Dg ;
        O O O       O       O    ;
        O O O       O       O];
    
    % Noise gain matrix
    Gc=[O O; Rb2t O ; O -Rb2t; O O; O O];
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    % Bias errors included
    Fc=[O I O O O;
        O O St Rb2t O;
        O O O O -Rb2t;
        O O O B1 O;
        O O O O B2];
    
    % Noise gain matrix
    Gc=[O O O O; Rb2t O O O; O -Rb2t O O; O O I O; O O O I];
    
else
    Fc=[O I O;
        O O St;
        O O O];
    
    % Noise gain matrix
    Gc=[O O; Rb2t O; O -Rb2t];
    
end

% Approximation of the discret time state transition matrices
F=eye(size(Fc))+simdata.Ts*Fc;
G=simdata.Ts*Gc;
end

function [x_out q_out]=comp_internal_states(x_in,dx,q_in)

% Convert quaternion to a rotation matrix
R=q2dcm(q_in);

% Correct the state vector
x_out=x_in+dx;

% Correct the rotation matrics
epsilon=dx(7:9);
OMEGA=[0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
R=(eye(3)-OMEGA)*R;

% Get the corrected roll, pitch and heading from the corrected rotation
% matrix
x_out(7)=atan2(R(3,2),R(3,3));
x_out(8)=-atan(R(3,1)/sqrt(1-R(3,1)^2));
x_out(9)=atan2(R(2,1),R(1,1));

% Calculte the corrected quaternions
q_out=dcm2q(R);

end

function u_out=comp_imu_errors(u_in,x_h)

global simdata;

% Check which errors that are included in the state space model

if (strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'on'))
    
    % Both scale and bias errors included
    temp=1./(ones(6,1)-x_h(16:end));
    u_out=diag(temp)*u_in+x_h(10:15);
    
    
elseif strcmp(simdata.scalefactors,'on') && strcmp(simdata.biases,'off')
    
    % Scale errors included
    temp=1./(ones(6,1)-x_h(10:end));
    u_out=diag(temp)*u_in;
    
elseif strcmp(simdata.scalefactors,'off') && strcmp(simdata.biases,'on')
    
    % Bias errors included
    u_out=u_in+x_h(10:end);
    
else
    
    % Only the standard errors included
    u_out=u_in;
end

end

function R=q2dcm(q)

p=zeros(6,1);

p(1:4)=q.^2;

p(5)=p(2)+p(3);

if p(1)+p(4)+p(5)~=0
   p(6)=2/(p(1)+p(4)+p(5)); 
else
   p(6)=0;
end


R(1,1)=1-p(6)*p(5);
R(2,2)=1-p(6)*(p(1)+p(3));
R(3,3)=1-p(6)*(p(1)+p(2));

p(1)=p(6)*q(1); 
p(2)=p(6)*q(2);
p(5)=p(6)*q(3)*q(4);
p(6)=p(1)*q(2);

R(1,2)=p(6)-p(5);
R(2,1)=p(6)+p(5);

p(5)=p(2)*q(4);
p(6)=p(1)*q(3);

R(1,3)=p(6)+p(5);
R(3,1)=p(6)-p(5);

p(5)=p(1)*q(4);
p(6)=p(2)*q(3);

R(2,3)=p(6)-p(5);
R(3,2)=p(6)+p(5);

end

function q=dcm2q(R)

T = 1 + R(1,1) + R(2,2) + R(3,3);

if T > 10^-8
    
    S = 0.5 / sqrt(T);
    qw = 0.25 / S;
    qx = ( R(3,2) - R(2,3) ) * S;
    qy = ( R(1,3) - R(3,1) ) * S;
    qz = ( R(2,1) - R(1,2) ) * S;

else
    
    if (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        
        S = sqrt( 1 + R(1,1) - R(2,2) - R(3,3)) * 2; % S=4*qx
        qw = (R(3,2) - R(2,3)) / S;
        qx = 0.25 * S;
        qy = (R(1,2) + R(2,1)) / S;
        qz = (R(1,3) + R(3,1)) / S;
        
    elseif (R(2,2) > R(3,3))
        
        S = sqrt( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2; %S=4*qy
        qw = (R(1,3) - R(3,1)) / S;
        qx = (R(1,2) + R(2,1)) / S;
        qy = 0.25 * S;
        qz = (R(2,3) + R(3,2)) / S;

    else
        
        S = sqrt( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2; % S=4*qz
        qw = (R(2,1) - R(1,2)) / S;
        qx = (R(1,3) + R(3,1)) / S;
        qy = (R(2,3) + R(3,2)) / S;
        qz = 0.25 * S;

    end

end

%Store in vector
q = [qx qy qz qw]';
end

function R=Rt2b(ang)


cr=cos(ang(1));
sr=sin(ang(1));

cp=cos(ang(2));
sp=sin(ang(2));

cy=cos(ang(3));
sy=sin(ang(3));

R=[cy*cp sy*cp -sp; 
    -sy*cr+cy*sp*sr cy*cr+sy*sp*sr cp*sr; 
    sy*sr+cy*sp*cr -cy*sr+sy*sp*cr cp*cr];

end

function [x_h P quat]=MeasurementUpdate(pseudo_measurement, H, x_h, P, quat, R)

% Calculate the Kalman filter gain
K = (P*H')/(H*P*H'+R);

% Calculate the prediction error.
z = pseudo_measurement - H*x_h;

% Estimation of the perturbations in the estimated navigation states
dx = K*z;

% Correct the navigation state using the estimated perturbations. 
[x_h(1:9) quat] = comp_internal_states(x_h(1:9), dx, quat);     % Subfunction located further down in the file.
        
% Update the filter state covariance matrix P.
Id = eye(size(P));
P = (Id - K*H)*P;
        
% Make sure the filter state covariance matrix is symmetric. 
P=(P+P')/2;     

end

function [x_h P quat] = MeasurementUpdate_Heading(heading, x_h, P, quat)

R=0.1;
H = zeros(1,9);
H(9)=1;
K = (P*H')/(H*P*H'+R);

z = heading;

x_h = x_h + K*(z - H*x_h);


% epsilon=[0 0 dx(9)];
% OMEGA=[0 -epsilon(3) epsilon(2); epsilon(3) 0 -epsilon(1); -epsilon(2) epsilon(1) 0];
% R=(eye(3)-OMEGA)*R;

% x_h(7)=atan2(R(3,2),R(3,3));
% x_h(8)=-atan(R(3,1)/sqrt(1-R(3,1)^2));
%x_h(9)=atan2(R(2,1),R(1,1));
R=Rt2b([x_h(7) x_h(8) x_h(9)]);
quat=dcm2q(R);
%
        
Id = eye(size(P));
P = (Id - K*H)*P;
        
P=(P+P')/2;  

end
