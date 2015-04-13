function [P0, P1,P2,P3,P4 ] = getWC( id )
%outputs the p0, p1, p2, p3 ,p4 in world frame for any tag id

%P0 = [x;   P1 = [x1;
%      y]         y1]...

%%decompose id into row & column
%12x9 grid: distances (in [m]) are from top left corner
[row,col] = ind2sub([12,9],id);

%% Algorithm to find distance of top left corner to origin
dx = 0.152; % in [m]
large_dx = 0.178; % in [m]
% x
x = 2*(row-1)*dx;

% y
if (col<4)
  y = 2*(col-1)*dx;% +dx
elseif (col>=4) && (col<7)
    % one space is now 0.178 not 0.152. just correct by adding the difference
  y = 2*(col-1)*dx  +(-dx + large_dx); %+dx
else
    % two spaces are now 0.178 not 0.152. just correct by adding the difference
  y = 2*(col-1)*dx + 2*(-dx + large_dx);%+dx
end


%% can now calc all corners
%         Y
%         ^ P3 == P2
%         | || P0 ||
%         | P4 == P1
%         o---------> X
% P4 = (x;y)
% P3 = (x; y + 0.152)
% P2 = (x + 0.152; y + 0.152)
% P1 = (x + 0.152; y)
% P0 = (x + 0.152/2; y + 0.152/2)

P4 = [x;y];
P3 = [x; y + dx];
P2 = [x + dx; y + dx];
P1 = [x + dx; y];
P0 = [x + dx/2; y + dx/2];


    



end

