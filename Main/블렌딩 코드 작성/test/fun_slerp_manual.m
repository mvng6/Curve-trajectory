function q_interp = fun_slerp_manual(q_start, q_end, t)
    % 쿼터니언 구면 선형 보간(SLERP)을 수동으로 구현하는 함수
    % 입력:
    %   q_start: 시작 쿼터니언 (1x4 벡터)
    %   q_end:   끝 쿼터니언 (1x4 벡터)
    %   t:       보간 계수 (0에서 1 사이의 스칼라 또는 벡터)

    % 두 쿼터니언 사이의 각도의 코사인 값 계산
    cos_omega = dot(q_start, q_end);

    % 항상 최단 경로로 보간하기 위해, cos_omega가 음수이면 q_end를 반전
    if cos_omega < 0
        q_end = -q_end;
        cos_omega = -cos_omega;
    end
    
    % 두 쿼터니언이 매우 가까울 경우 (선형 보간으로 근사)
    if (1.0 - cos_omega) > 1e-6
        omega = acos(cos_omega);
        sin_omega = sin(omega);
        scale0 = sin((1.0 - t) * omega) / sin_omega;
        scale1 = sin(t * omega) / sin_omega;
    else
        % 각도가 매우 작으면 선형 보간(LERP)을 사용 (안정성)
        scale0 = 1.0 - t;
        scale1 = t;
    end
    
    % 보간된 쿼터니언 계산
    q_interp = scale0' .* q_start + scale1' .* q_end;
    
    % 최종 쿼터니언 정규화
    q_interp = q_interp ./ vecnorm(q_interp, 2, 2);
end