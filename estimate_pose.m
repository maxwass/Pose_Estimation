function [pos, eul] = estimate_pose(sensor, varargin)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid

%          - rpy, omg, acc: imu readings, you should not use these in this phase

%          - img: uint8, 240x376 grayscale image


%          - id: 1xn ids of detected tags

%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X

%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   eul - 3x1 euler angles of the quadrotor

pos = zeros(3,1);
eul = zeros(3,1);

%Rotation matrix to put world into Camera frame

Rotation_C2B = [ cosd(45)  -sind(45)  0;
                -sind(45)  -cosd(45)  0;
                   0          0       -1];
               
%Rotation_C2B = [ cosd(-45)  -sind(-45)  0; 
%                 sind(-45)    cosd(-45)  0
%                 0           0         -1];
             
%Translation from camera to body frame
Translation_C2B = [ -.04;
                      0;
                    -.03];

homogRotTran_C2B = [Rotation_C2B Translation_C2B;
                     0 0 0               1       ]; 

homogRotTran_B2C = inv(homogRotTran_C2B);
  
%Camera Calibration Matrix
K = [314.1779 0         199.4848;
0         314.2218  113.7838;
0         0         1];


%If there are none return empty
if (isempty(sensor.id))
    pos = [];
    eul = [];
    return
else

%these are the april tags we see
april_tag_ids = sensor.id +1;

A = zeros(10*length(april_tag_ids),9);  %%?? correct size

for jj = 1:length(april_tag_ids)
    
    %get corner coordinates: X,Y in world
    [p0, p1, p2, p3, p4] = getWC(april_tag_ids(jj));
    
    %get pixel coordinates
    [p0p, p1p, p2p, p3p, p4p] = getPC(sensor,jj);
            
    %begin constructing matrix
    [ subA ] = findsubA([p0, p1, p2, p3, p4], [p0p, p1p, p2p, p3p, p4p]);
    
    index = 10*(jj-1) + 1;
    A(index:index+9, :) = subA;
    
end
                
[~,~,V] = svd(A);
soln = V(:,end); %this is the soln to [r11 r12....]

R1 = soln(1:3);
R2 = soln(4:6);
T  = soln(7:9);

H_pixel  = [R1, R2, T];

%get into camera frame
H_camera = inv(K)*H_pixel;

%normalize
H_camera_norm = H_camera./H_camera(3,3);

%force it to be valid rotation matrix
[Uc,~,Vc] = svd([H_camera_norm(:,1),H_camera_norm(:,2),cross(H_camera_norm(:,1),H_camera_norm(:,2))]);
VcT = Vc';

Rotation_W2C = Uc*[1 0 0; 0 1 0; 0 0 det(Uc*VcT)]*VcT;

Translation_W2C = H_camera_norm(:,3)./norm(H_camera_norm(:,1));

homogRotTran_W2C = [Rotation_W2C Translation_W2C;
                     0 0 0               1       ];
            
W2B =  homogRotTran_B2C*homogRotTran_W2C;

Rot_W2B = W2B(1:3,1:3); 
Trans_W2B = W2B(1:3,4);

pos = (-Rot_W2B')*(Trans_W2B);
pos = pos';

[phi, theta, psi] = RotToRPY_ZXY(Rot_W2B);

eul = [phi, theta, psi];

end


end
