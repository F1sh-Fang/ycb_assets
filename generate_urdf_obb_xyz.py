import os
import trimesh
import numpy as np

# --- 配置区域 ---
BASE_YCB_PATH = "ycb_assets"
VISUAL_MESH_FILENAME = "textured.obj"
COLLISION_MESH_FILENAME = "textured_vhacd.obj"
OUTPUT_NPZ_FILE = "ycb_obb_extents.npz"
# --- 配置结束 ---

# URDF模板
URDF_TEMPLATE_BASE_ROOT = """<?xml version="1.0"?>
<robot name="{object_name}">
  
  <link name="base_link">
    <inertial>
      <origin xyz="{com_origin_in_base_str}" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="{visual_origin_xyz_str}" rpy="{visual_origin_rpy_str}"/>
      <geometry>
        <mesh filename="{visual_mesh}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="{visual_origin_xyz_str}" rpy="{visual_origin_rpy_str}"/>
      <geometry>
        <mesh filename="{collision_mesh}"/>
      </geometry>
    </collision>
  </link>

  <link name="center_of_mass"/>

  <link name="obb_center"/>

  <joint name="base_to_com" type="fixed">
    <parent link="base_link"/>
    <child link="center_of_mass"/>
    <origin xyz="{com_origin_in_base_str}" rpy="0 0 0"/>
  </joint>

  <joint name="base_to_obb" type="fixed">
    <parent link="base_link"/>
    <child link="obb_center"/>
    <origin xyz="{obb_origin_in_base_str}" rpy="0 0 0"/>
  </joint>

</robot>
"""

def get_orientation_and_extents_from_projection(mesh):
    """
    【最终正确版+尺寸输出】通过PCA分析物体在XY平面的投影，计算旋转矩阵和[长,宽,高]尺寸。
    """
    # 步骤 1: 获取3D OBB的8个角点
    obb = mesh.bounding_box_oriented
    obb_vertices_3d = obb.vertices

    # 步骤 2: 将角点投影到XY平面
    points_2d = obb_vertices_3d[:, :2]

    # 步骤 3: 对2D点集执行PCA
    centered_points = points_2d - np.mean(points_2d, axis=0)
    covariance_matrix = np.cov(centered_points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance_matrix)
    
    # 步骤 4: 构建新的、绝对一致的旋转矩阵 R_final
    y_direction_2d = eigenvectors[:, -1]
    y_final = np.array([y_direction_2d[0], y_direction_2d[1], 0.0])
    z_final = np.array([0.0, 0.0, 1.0])
    x_final = np.cross(y_final, z_final)
    R_final = np.column_stack([x_final, y_final, z_final])

    # 步骤 5: 【新增】计算与新坐标系对应的尺寸
    # a. 将OBB的8个角点，变换到这个新的坐标系下
    #    变换是通过乘以旋转矩阵的逆（也就是转置）
    transformed_vertices = obb_vertices_3d @ R_final
    
    # b. 计算变换后角点的轴对齐边界框(AABB)的最小和最大点
    min_coords = np.min(transformed_vertices, axis=0)
    max_coords = np.max(transformed_vertices, axis=0)
    
    # c. AABB的尺寸就是最大和最小坐标的差
    aabb_extents = max_coords - min_coords

    # d. 按照[长, 宽, 高]的顺序组合最终尺寸
    #    我们的Y轴是长边，X轴是宽边，Z轴是高
    final_extents = np.array([aabb_extents[1], aabb_extents[0], aabb_extents[2]])
    
    return R_final, final_extents

def generate_urdf_files_final():
    print("--- 开始生成URDF文件并提取OBB尺寸 (最终PCA版) ---")
    if not os.path.isdir(BASE_YCB_PATH):
        print(f"错误: 目录 '{BASE_YCB_PATH}' 不存在。")
        return

    all_obb_extents = {}

    object_folders = sorted([d for d in os.listdir(BASE_YCB_PATH) if os.path.isdir(os.path.join(BASE_YCB_PATH, d))])
    
    for folder_name in object_folders:
        if not folder_name[:3].isdigit() or '_' not in folder_name:
            continue
        
        object_name = folder_name
        target_dir = os.path.join(BASE_YCB_PATH, folder_name, "google_16k")
        if not os.path.isdir(target_dir):
            continue

        print(f"正在处理对象: {object_name}")

        collision_mesh_path = os.path.join(target_dir, COLLISION_MESH_FILENAME)
        if not os.path.exists(collision_mesh_path):
            print(f"  -> 警告: 碰撞模型不存在，已跳过。")
            continue
        
        try:
            mesh = trimesh.load(collision_mesh_path, force='mesh', process=False)
        except Exception as e:
            print(f"  -> 错误: 加载模型失败: {e}")
            continue

        com_original = mesh.centroid
        base_link_original_pos = np.array([com_original[0], com_original[1], 0.0])
        
        obb = mesh.bounding_box_oriented
        obb_center_original_pos = obb.centroid
        
        # 使用我们最终的、基于PCA的函数
        aligned_rotation, final_extents = get_orientation_and_extents_from_projection(mesh)

        all_obb_extents[object_name] = final_extents
        print(f"  -> 提取尺寸 [长, 宽, 高]: {np.array2string(final_extents, precision=4)}")

        T_base_new = np.eye(4)
        T_base_new[:3, :3] = aligned_rotation
        T_base_new[:3, 3] = base_link_original_pos
        T_base_new_inv = np.linalg.inv(T_base_new)

        com_in_base = T_base_new_inv @ np.append(com_original, 1)
        obb_in_base = T_base_new_inv @ np.append(obb_center_original_pos, 1)
        
        com_origin_in_base_str = " ".join(f"{x:.6f}" for x in com_in_base[:3])
        obb_origin_in_base_str = " ".join(f"{x:.6f}" for x in obb_in_base[:3])

        visual_transform = T_base_new_inv
        visual_origin_xyz = visual_transform[:3, 3]
        visual_origin_rpy = trimesh.transformations.euler_from_matrix(visual_transform[:3, :3], 'sxyz')
        
        visual_origin_xyz_str = " ".join(f"{x:.6f}" for x in visual_origin_xyz)
        visual_origin_rpy_str = " ".join(f"{x:.6f}" for x in visual_origin_rpy)

        urdf_content = URDF_TEMPLATE_BASE_ROOT.format(
            object_name=object_name,
            visual_mesh=VISUAL_MESH_FILENAME,
            collision_mesh=COLLISION_MESH_FILENAME,
            com_origin_in_base_str=com_origin_in_base_str,
            obb_origin_in_base_str=obb_origin_in_base_str,
            visual_origin_xyz_str=visual_origin_xyz_str,
            visual_origin_rpy_str=visual_origin_rpy_str,
        )
        
        output_urdf_path = os.path.join(target_dir, f"{object_name}.urdf")
        try:
            with open(output_urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"  -> 已成功生成URDF文件: {output_urdf_path}")
        except IOError as e:
            print(f"  -> 错误: 无法写入文件。原因: {e}")

    if all_obb_extents:
        try:
            np.savez_compressed(OUTPUT_NPZ_FILE, **all_obb_extents)
            print(f"\n✅ 所有物体的OBB尺寸已成功保存到: '{OUTPUT_NPZ_FILE}'")
            print("   加载方法: data = np.load('ycb_obb_extents.npz')")
            print("   示例访问: data['004_sugar_box'] -> array([length, width, height])")
        except Exception as e:
            print(f"\n❌ 错误: 保存 .npz 文件时出错: {e}")
    else:
        print("\n没有处理任何物体，未生成 .npz 文件。")

if __name__ == "__main__":
    generate_urdf_files_final()