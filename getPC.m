function [p0p, p1p, p2p, p3p, p4p] = getPC(sensor, index)

p0p = sensor.p0(:,index);
p1p = sensor.p1(:,index);
p2p = sensor.p2(:,index);
p3p = sensor.p3(:,index);
p4p = sensor.p4(:,index);


end

