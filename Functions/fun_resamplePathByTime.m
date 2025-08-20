function [resampled_points, resampled_normals, total_travel_time] = fun_resamplePathByTime(original_points, original_normals, speed_mmps, time_step)
    % 경로의 누적 거리 계산 (단위: mm)
    distances = vecnorm(diff(original_points), 2, 2);
    cumulative_dist = [0; cumsum(distances)];
    total_path_length = cumulative_dist(end);

    % 시간 기반으로 리샘플링할 새로운 지점들의 거리 정의
    total_travel_time = total_path_length / speed_mmps;
    new_sample_times = (0:time_step:total_travel_time)';
    new_sample_dists = new_sample_times * speed_mmps;

    % 경로의 끝점이 누락되지 않도록 보장
    if new_sample_dists(end) < total_path_length
        new_sample_dists(end+1) = total_path_length;
    end

    % 선형 보간을 이용한 위치 및 법선 벡터 리샘플링
    resampled_points(:,1) = interp1(cumulative_dist, original_points(:,1), new_sample_dists, 'spline');
    resampled_points(:,2) = interp1(cumulative_dist, original_points(:,2), new_sample_dists, 'spline');
    resampled_points(:,3) = interp1(cumulative_dist, original_points(:,3), new_sample_dists, 'spline');
    
    resampled_normals(:,1) = interp1(cumulative_dist, original_normals(:,1), new_sample_dists, 'spline');
    resampled_normals(:,2) = interp1(cumulative_dist, original_normals(:,2), new_sample_dists, 'spline');
    resampled_normals(:,3) = interp1(cumulative_dist, original_normals(:,3), new_sample_dists, 'spline');
    
    % 보간된 법선 벡터 정규화
    resampled_normals = resampled_normals ./ vecnorm(resampled_normals, 2, 2);
end