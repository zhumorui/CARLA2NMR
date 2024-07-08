import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from src.utils.colmap.python.read_write_model import read_model, write_model, qvec2rotmat, rotmat2qvec

# 读取gt轨迹文件内容
def read_gt_trajectory(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    poses = []
    for line in lines:
        values = line.strip().split()
        if not values:  # 跳过空行
            continue
        if len(values) != 10:
            print(f"Warning: Skipping line due to insufficient values!")
            continue
        try:
            idx = int(values[0])
            qvec = list(map(float, values[1:5]))
            tvec = list(map(float, values[5:8]))
            poses.append((qvec, tvec))
        except ValueError as e:
            print(f"Error parsing line: {line}\n{e}")
    
    return poses

# 读取est轨迹文件内容
def read_est_trajectory(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    transformations = []
    for line in lines:
        values = line.strip().split()
        if not values:  # 跳过空行
            continue
        values = list(map(float, values))
        if len(values) != 12:
            print(f"Warning: Skipping line due to incorrect number of values!")
            continue
        try:
            matrix = np.array(values).reshape(3, 4)
            
            # 4x4 transformation
            matrix = np.vstack((matrix, [0, 0, 0, 1]))

            transformations.append(matrix)
        except ValueError as e:
            print(f"Error parsing line: {line}\n{e}")
    
    return transformations

# 将四元数和位置转换为4x4变换矩阵
def get_transform(pose):
    qvec, tvec = pose
    R = qvec2rotmat(qvec)
    t = np.array(tvec).reshape(3, 1)

    # invert
    t = -R.T @ t
    R = R.T
    
    # 4x4 transformation
    T = np.column_stack((R, t))
    T = np.vstack((T, [0, 0, 0, 1]))

    return T

def align_trajectories(gt_poses, est_poses):
    aligned_est_poses = est_poses

    # Optional: Perform necessary point cloud transformation if needed
    # Reverse y
    # T = np.array([[1, 0, 0, 0],
    #               [0, -1, 0, 0],
    #               [0, 0, 1, 0],
    #               [0, 0, 0, 1]])
    # aligned_est_poses = [T @ pose for pose in aligned_est_poses]

    # # (x, y, z) -> (z, -x, -y) coordinate transformation
    # T = np.array([[0, -1, 0, 0],
    #               [0, 0, -1, 0],
    #               [1, 0, 0, 0],
    #               [0, 0, 0, 1]])
    # aligned_est_poses = [T @ pose for pose in aligned_est_poses]


    transform = get_transform(gt_poses[0])

    aligned_est_poses = [transform @ pose for pose in aligned_est_poses]

    
    # Keep Ground Truth poses as they are
    aligned_gt_poses = [get_transform(pose) for pose in gt_poses]

    print(aligned_est_poses[0])
    print(aligned_gt_poses[0])

    
    return aligned_gt_poses, aligned_est_poses

# 可视化轨迹
def visualize_trajectories(gt_poses, est_poses):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    
    for transform in gt_poses:
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        mesh.transform(transform)
        vis.add_geometry(mesh)

    for transform in est_poses:
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        mesh.transform(transform)
        vis.add_geometry(mesh)
    
    vis.run()
    vis.destroy_window()

# 计算绝对轨迹误差（ATE）
def compute_ate(gt_poses, est_poses):
    errors = []
    for gt, est in zip(gt_poses, est_poses):
        gt_translation = gt[:3, 3]
        est_translation = est[:3, 3]
        error = np.linalg.norm(gt_translation - est_translation)
        errors.append(error)
    return np.mean(errors), np.std(errors)

# 计算相对位姿误差（RPE）
def compute_rpe(gt_poses, est_poses, delta=1):
    errors = []
    for i in range(len(gt_poses) - delta):
        gt_rel = np.linalg.inv(gt_poses[i]) @ gt_poses[i + delta]
        est_rel = np.linalg.inv(est_poses[i]) @ est_poses[i + delta]

        # 提取旋转和平移
        gt_rot = R.from_matrix(gt_rel[:3, :3])
        est_rot = R.from_matrix(est_rel[:3, :3])
        gt_trans = gt_rel[:3, 3]
        est_trans = est_rel[:3, 3]

        rot_error = gt_rot.inv() * est_rot
        trans_error = np.linalg.norm(gt_trans - est_trans)
        errors.append((rot_error.magnitude(), trans_error))
    
    rot_errors, trans_errors = zip(*errors)
    return np.mean(rot_errors), np.std(rot_errors), np.mean(trans_errors), np.std(trans_errors)

# 导出对齐后的估计轨迹
def export_aligned_est_poses(file_path, aligned_est_poses):
    with open(file_path, 'w') as file:
        for id, pose in enumerate(aligned_est_poses):
            idx = id + 1
            camera = '1'
            img = f"{idx:04d}.png"
            qvec = rotmat2qvec(pose[:3, :3].T)
            tvec = pose[:3, 3]
            flattened_pose = np.concatenate((qvec, tvec))
            line = f"{idx} {' '.join(map(str, flattened_pose))} {camera} {img}"
            file.write(line + '\n')
            file.write('\n')

# 主函数
def main(gt_file, est_file, aligned_est_file):
    gt_poses = read_gt_trajectory(gt_file)
    est_poses = read_est_trajectory(est_file)
    
    gt_poses_aligned, est_poses_aligned = align_trajectories(gt_poses, est_poses)
    
    visualize_trajectories(gt_poses_aligned, est_poses_aligned)
    
    ate_mean, ate_std = compute_ate(gt_poses_aligned, est_poses_aligned)
    rpe_rot_mean, rpe_rot_std, rpe_trans_mean, rpe_trans_std = compute_rpe(gt_poses_aligned, est_poses_aligned)

    print(f"ATE Mean: {ate_mean}, ATE Std: {ate_std}")
    print(f"RPE Rotation Mean: {rpe_rot_mean}, RPE Rotation Std: {rpe_rot_std}")
    print(f"RPE Translation Mean: {rpe_trans_mean}, RPE Translation Std: {rpe_trans_std}")
    
    export_aligned_est_poses(aligned_est_file, est_poses_aligned)

# 文件路径
gt_file_path = 'data/carla_12_20_rgb_1_1/sparse/0/images.txt'
est_file_path = 'results/latest/lidar_poses_kitti.txt'
aligned_est_file_path = 'aligned_est_poses.txt'

# 运行主函数
main(gt_file_path, est_file_path, aligned_est_file_path)
