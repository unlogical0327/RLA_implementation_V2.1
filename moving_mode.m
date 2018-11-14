% this program handle the moving mode of AGV with a kine
function [moving_status,Lidar_trace] = moving_mode(moving_thres,amp_thres,reflector_diameter,distance_delta,Reflector_map,Reflector_ID,calibration_data,scan_data,thres_dist_match,thres_dist_large)

last_reflector(1,1)=match_reflect_pool(match_reflect_ID(1,end),1); % load the last reflector x
last_reflector(1,2)=match_reflect_pool(match_reflect_ID(1,end),2); % load the last reflector y
[pose_expect]=pose_expectation(pose_current, pose_hist);
x_increment=pose_expect(1,1)-post_hist(end,1);
y_increment=pose_expect(1,2)-post_hist(end,2);
expect_reflector(1,1)=last_reflector(1,1)-x_increment; % calculate the last reflector x and y and compare with measured last reflector
expect_reflector(1,2)=last_reflector(1,2)-y_increment;
% find the last reflector in scan and calculate 
[detected_ID,detected_reflector]=identify_reflector(amp_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
zz=length(detected_ID);
b=1; % flag to search for match reflector
%-- search for matched reflector from the last one
while b==1
last_scan_reflector=detected_reflector(detected_ID(zz));
x_delta=expect_reflector(1,1)-last_scan_reflector(1,1);
y_delta=expect_reflector(1,2)-last_scan_reflector(1,2);
if x_delta<moving_thres && y_delta<moving_thres
    moving_status=0
    disp('last reflector MATCH with expected position!')
    Lidar_update_xy=[last_scan_reflector(1,1) last_scan_reflector(1,2)];
    Lidar_trace=[Lidar_trace;Lidar_update_xy];
else
    moving_status=1;
    disp('last reflector NOT match with expected position!')
    zz=zz-1;
end
if moving_status==1 && zz=0
    b=0;
    break
end
end