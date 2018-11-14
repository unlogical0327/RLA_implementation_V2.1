
function [matched_reflect_ID,matched_reflect_vec_ID,matched_detect_ID,matched_detect_vec_ID,result] = match_reflector(match_dist_vector_pool,Reflect_vec_ID,detect_Ref_dist_vector,detected_vec_ID,thres_dist_large,thres_dist_match)
% NP_enable?
% function [matched_reflect_ID] = match_reflector(match_dist_vector_pool,Reflect_vec_ID,detect_Ref_dist_vector,detect_vect_ID,thres_dist_match,NP_enable)
% Define matching threshold value here
%-- match the distance matrix with reflector tables
matched_reflect_vec_ID=0;
matched_detect_vec_ID=0;
matched_reflect_ID=0;
matched_detect_ID=0;
m=0;n=0;

matched_reflect_vec_ID=0;
matched_detect_vec_ID=0;
% -- Need to filter the distance larger than certain threshold

for i=1:length(detect_Ref_dist_vector)
    distance_min=thres_dist_match+100;
    for j=1:length(match_dist_vector_pool)      
        if (match_dist_vector_pool(1,j)<thres_dist_large && detect_Ref_dist_vector(1,i)<thres_dist_large)
                distance_delta=abs(match_dist_vector_pool(1,j)-detect_Ref_dist_vector(1,i));
                if distance_delta<distance_min
                    distance_min=distance_delta;
                    Reflect_min_ID(1,1)=Reflect_vec_ID(j,1);
                    Reflect_min_ID(1,2)=Reflect_vec_ID(j,2);
                    detect_min_ID(1,1)=detected_vec_ID(i,1);
                    detect_min_ID(1,2)=detected_vec_ID(i,2);
                else
                    distance_min=distance_min;
                end
        end
    end
     if distance_min<thres_dist_match
                m=m+1;
                n=n+1;
                matched_reflect_vec_ID(m,1)=Reflect_min_ID(1,1);  %
                matched_reflect_vec_ID(m,2)=Reflect_min_ID(1,2);
                matched_detect_vec_ID(n,1)=detect_min_ID(1,1);
                matched_detect_vec_ID(n,2)=detect_min_ID(1,2);
                disp(sprintf('Matched reflector_vec ID: %i', j));
                disp(sprintf('Matched detect_vec ID: %i', i));
            end
end
(match_dist_vector_pool)
(detect_Ref_dist_vector)
matched_reflect_vec_ID
matched_detect_vec_ID
m
n
%if matched_reflect_vec_ID==0
%if m<=length(detect_Ref_dist_vector) && m>=3  % need at least 3 reflectors
if  m>=3
    matched_reflect_ID=unique(matched_reflect_vec_ID)';
    matched_detect_ID=unique(matched_detect_vec_ID)';
    if length(matched_reflect_ID)~=length(matched_detect_ID)
        result=1;
    else   
    disp('matched reflectors: ');
    disp(sprintf('Reflector ID:-%i ', matched_reflect_ID));
    result=0;
    end
elseif m<=2   %length(detect_Ref_dist_vector)
    disp('Not enough matched reflectors found');
    result=1;
    return
end
