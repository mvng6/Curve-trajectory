function [blended_path, blended_normals] = fun_apply_path_blending_normals(original_path, original_normals, corner_indices, blend_dist)
    % 탐지된 모든 코너 인덱스에 대해 위치와 법선 벡터 블렌딩을 순차적으로 적용

    blended_path = original_path;
    blended_normals = original_normals; % 법선 벡터도 초기화

    % 경로가 변경되는 것을 고려하여, 인덱스가 꼬이지 않도록 역순으로 처리
    for i = length(corner_indices):-1:1
        corner_idx = corner_indices(i);
        fprintf('--- 인덱스 %d 코너에 블렌딩 적용 ---\n', corner_idx);
        
        % 위치와 법선 벡터를 모두 전달하고 결과를 받음
        [blended_path, blended_normals] = fun_blend_single_corner_normals(blended_path, blended_normals, corner_idx, blend_dist);
    end
end