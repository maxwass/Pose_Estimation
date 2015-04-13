function [ subA ] = findsubA(worldCoord, pixelCoord)
%Outputs: the H matrix
%Inputs: world coord and normalized coordinates, contructs matrix, and
    %solves it

%form: ' = norm, 2x9
% [-x -y 0 0  xx' yx' -1 0  0;
% [0  0 -x -y xy' yy'  0 -1 0]
% to solve for: 9x1
%[r11 r12 r21 r22 r31 r32 t1 t2 t3]'
% first 2x9 matrix for point 1
coef = zeros(10,9);

for jj = 1:5

%pull out x,y, xnorm, ynorm
wc = worldCoord(:,jj);
x = wc(1);
y = wc(2);

pc = pixelCoord(:,jj);
xp = pc(1);
yp = pc(2);
   
c = [-x  0 x*xp  -y  0  y*xp  -1  0  xp;
     0  -x x*yp   0  -y y*yp  0  -1  yp]; 
   
index = 2*(jj-1) + 1;
coef(index:index+1,:) =  c;
       
end

subA = coef;

end