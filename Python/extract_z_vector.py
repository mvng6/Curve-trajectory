import trimesh
import numpy as np
import pandas as pd

# 1. STL 파일 로드
try:
    mesh = trimesh.load_mesh('TEST_WAVE_1.stl')
    print(f"메시 로드 성공: {len(mesh.vertices)}개의 꼭짓점, {len(mesh.faces)}개의 면")
except Exception as e:
    print(f"파일을 로드하는 데 실패했습니다: {e}")
    exit()

# 2. 광선 생성 (## 핵심 변경 사항 ##)

# 모델의 경계(bounds) 확인
min_bounds, max_bounds = mesh.bounds

# ## 수정된 부분 1 ##
# Y축을 따라 샘플링할 지점들을 생성합니다.
# 예를 들어, y축 시작점부터 끝점까지 100개의 점을 샘플링
y_coords = np.linspace(min_bounds[1], max_bounds[1], 100)

# ## 수정된 부분 2 ##
# X좌표는 모델의 중앙으로 고정 (필요에 따라 특정 값으로 변경 가능)
x_coord = (min_bounds[0] + max_bounds[0]) / 2

# ## 수정된 부분 3 ##
# 광선의 원점들: Y좌표가 변하고 X좌표는 고정됩니다.
# z좌표는 모델의 가장 높은 지점보다 약간 위에서 시작하여 아래로 쏩니다.
ray_origins = np.array([
    [x_coord, y, max_bounds[2] + 1] for y in y_coords
])

# 광선의 방향: 모두 Z축의 음수 방향 (-Z)으로 설정 (이 부분은 변경 없음)
ray_directions = np.array([[0, 0, -1]] * len(ray_origins))


# 3. 광선-메시 교차 계산 (변경 없음)
locations, index_ray_loc, index_tri_loc = mesh.ray.intersects_location(
    ray_origins=ray_origins,
    ray_directions=ray_directions
)

print(f"\n총 {len(ray_origins)}개의 광선을 쏘아 {len(locations)}개의 교차점을 찾았습니다.")

# 4. 인덱스를 활용한 데이터 정리 및 출력 (변경 없음)
results = []
for i in range(len(locations)):
    path_index = i
    pos = locations[i]
    face_index = index_tri_loc[i]
    normal = mesh.face_normals[face_index]
    vertex_indices_of_face = mesh.faces[face_index]
    
    results.append({
        "Path_Index": path_index,
        "X": pos[0], "Y": pos[1], "Z": pos[2],
        "Normal_X": normal[0], "Normal_Y": normal[1], "Normal_Z": normal[2],
        "Hit_Face_Index": face_index,
        "V1_Index": vertex_indices_of_face[0],
        "V2_Index": vertex_indices_of_face[1],
        "V3_Index": vertex_indices_of_face[2]
    })

# 5. 최종 결과를 표(DataFrame)로 변환하여 확인 및 저장 (변경 없음)
df = pd.DataFrame(results)
print("\n--- 추출된 데이터 (상위 5개) ---")
print(df.head())

df.to_csv('indexed_path_data_Y_axis.csv', index=False)
print("\n결과를 'indexed_path_data_Y_axis.csv' 파일로 저장했습니다.")