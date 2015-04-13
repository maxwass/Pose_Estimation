%test script

%1
load('studentdata1.mat')
sensor = data(1);
[pos, eul] = estimate_pose(sensor);
x = vicon(1,1); 
y = vicon(2,1);
z =  vicon(3,1); 
pos_truth = [x,y,z];
roll = vicon(4,1);
pitch = vicon(5,1);
yaw = vicon(6,1); 
eul_truth = [roll, pitch, yaw];

pos_error = pos_truth - pos
eul_error = eul_truth - eul

%compare to vicon....
clear data time vicon

%2
load('studentdata4.mat')


%compare to vicon....
clear data time vicon

%3
load('studentdata1.mat')
data(1).id


%compare to vicon....
clear data time vicon