% function to generate pose expectation from the previous moment pose information
% pose_expectation and pose_current are 3x1 matrix contains x, y and theta  
function [pose_expect]=pose_expectation(pose_current, pose_his)
l = length(pose_his);
%theta_delta = 
t=1/scan_freq;
x_move = pose_his(l,1)-pose_his(l-1,1);
y_move = pose_his(l,2)-pose_his(l-1,2);
pose_expect(1,3) = pose_current(l,3); % assume the angle is not changing during the moving
pose_expect(1,1) = pose_his(l,1)+x_move;
pose_expect(1,2) = pose_his(l,2)+y_move;

