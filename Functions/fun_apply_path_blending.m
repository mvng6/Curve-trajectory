function blended_path = fun_apply_path_blending(original_path, corner_indices, blending_config)
    % 탐지된 모든 코너 인덱스에 대해 블렌딩을 순차적으로 적용하는 함수
    
    blended_path = original_path;
    blend_dist = blending_config.distance;

    % 경로가 변경되는 것을 고려하여, 인덱스가 꼬이지 않도록 역순으로 처리
    for i = length(corner_indices):-1:1
        corner_idx = corner_indices(i);
        fprintf('--- 인덱스 %d 코너에 블렌딩 적용 (거리: %.1f) ---\n', corner_idx, blend_dist);
        blended_path = fun_blend_single_corner(blended_path, corner_idx, blend_dist);
    end
end