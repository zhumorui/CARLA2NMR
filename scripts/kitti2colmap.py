import numpy as np
from scipy.spatial.transform import Rotation as R

# KITTI格式的数据
with open('results/2024-06-24_09-49-31/lidar_poses_kitti.txt', 'r') as f:
    kitti_data = f.readlines()
    f.close()

kitti_data = [line.strip().split() for line in kitti_data]


# 函数：从KITTI数据中提取旋转矩阵和平移向量，并将旋转矩阵转换为四元数
def kitti_to_colmap(kitti_data):
    colmap_data = []
    for i, kitti_line in enumerate(kitti_data):
        rotation_matrix = np.array(kitti_line[:9]).reshape(3, 3)
        translation_vector = np.array(kitti_line[9:]).reshape(3)
        
        # 将旋转矩阵转换为四元数
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # [qx, qy, qz, qw]
        
        # COLMAP需要的格式是 [qw, qx, qy, qz]
        quaternion = np.roll(quaternion, 1)
        
        # 将数据添加到结果中
        image_id = i + 1
        camera_id = 1  # 假设相机ID为1
        image_name = f"{i+1:04d}.png"  # 图像名从0001.png到0150.png
        colmap_data.append([
            image_id,
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
            translation_vector[0], translation_vector[1], translation_vector[2],
            camera_id,
            image_name
        ])
    return colmap_data

# 生成COLMAP格式的数据
colmap_data = kitti_to_colmap(kitti_data)

# 保存COLMAP格式的数据

with open('images.txt', 'w') as f:
    for line in colmap_data:
        f.write(' '.join(map(str, line)) + '\n')

        # write space 
        f.write('\n')
    f.close()
