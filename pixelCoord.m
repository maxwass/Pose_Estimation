function [p0n, p1n, p2n, p3n, p4n ] = pixelCoord(p0, p1, p2, p3, p4)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%Camera Calibration Matrix
K = [314.1779 0         199.4848;
0         314.2218  113.7838;
0         0         1];

invK = inv(K);

%pi is a 2x1 vector, need to make it a 3x1
p0 = vertcat(p0,1);
p1 = vertcat(p1,1);
p2 = vertcat(p2,1);
p3 = vertcat(p3,1);
p4 = vertcat(p4,1);

%hit with inv k to get normalized coordinates (these are the unscaled
%position in camera frame [m]
p0n = invK*po;
p1n = invK*p1;
p2n = invK*p2;
p3n = invK*p3;
p4n = invK*p4;

%take off last 1
p0n = p0n(1:2);
p1n = p1n(1:2);
p2n = p2n(1:2);
p3n = p3n(1:2);
p4n = p4n(1:2);


end