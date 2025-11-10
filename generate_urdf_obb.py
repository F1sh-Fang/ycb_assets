import os
import trimesh
import numpy as np

# --- 配置区域 ---
BASE_YCB_PATH = "ycb_assets"
VISUAL_MESH_FILENAME = "textured.obj"
COLLISION_MESH_FILENAME = "textured_vhacd.obj"
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

def get_z_up_orientation_from_obb(mesh):
    """
    【自实现最终版】从3D OBB结果中，计算出一个新的旋转矩阵。
    新矩阵的Z轴固定朝上，Y轴与物体在XY平面上的长边对齐。
    """
    # 1. 获取唯一的依赖：3D有向边界框
    obb = mesh.bounding_box_oriented
    obb_extents = obb.extents
    R_original = obb.transform[:3, :3]

    # 2. 找出 OBB 自身的三个轴中，哪两个在XY平面上的投影最大
    #    这可以避免选择那个几乎垂直于XY平面的轴
    #    我们通过看每个轴的Z分量来判断其“垂直程度”
    #    Z分量绝对值越小，说明这个轴越“平躺”在XY平面上
    z_components_abs = np.abs(R_original[2, :])
    # 找到最垂直的轴的索引 (我们想忽略它)
    vertical_axis_idx = np.argmax(z_components_abs)
    # 剩下的两个就是我们关心的“平面内”轴
    in_plane_indices = [i for i in range(3) if i != vertical_axis_idx]

    # 3. 在这两个“平面内”轴中，找到对应尺寸更长的那个
    extent1 = obb_extents[in_plane_indices[0]]
    extent2 = obb_extents[in_plane_indices[1]]
    
    long_axis_idx = in_plane_indices[0] if extent1 > extent2 else in_plane_indices[1]

    # 4. 构建新的坐标系
    # 新的Y轴是长轴在XY平面上的投影，并归一化
    y_candidate = R_original[:, long_axis_idx]
    y_new = np.array([y_candidate[0], y_candidate[1], 0.0])
    # 处理极端情况：如果长轴本身就是Z轴，投影会是零向量
    if np.linalg.norm(y_new) < 1e-6:
        y_new = np.array([1.0, 0.0, 0.0]) # 随便选一个方向，比如X轴
    else:
        y_new = y_new / np.linalg.norm(y_new)
        
    # 新的Z轴固定朝上
    z_new = np.array([0.0, 0.0, 1.0])
    
    # 新的X轴通过叉乘得到，确保是右手坐标系
    x_new = np.cross(y_new, z_new)

    # 5. 组合成最终的旋转矩阵
    R_final = np.column_stack([x_new, y_new, z_new])
    
    return R_final


def generate_urdf_files_z_up():
    print("--- 开始生成Z轴朝上、与长边对齐的URDF文件 (最终保证兼容版) ---")
    if not os.path.isdir(BASE_YCB_PATH):
        print(f"错误: 目录 '{BASE_YCB_PATH}' 不存在。")
        return

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

        # 1. 计算所有原始位置和新的姿态
        com_original = mesh.centroid
        base_link_original_pos = np.array([com_original[0], com_original[1], 0.0])
        
        obb = mesh.bounding_box_oriented
        obb_center_original_pos = obb.centroid
        
        # 核心：使用我们自己实现的、绝对可靠的函数获取旋转矩阵
        aligned_rotation = get_z_up_orientation_from_obb(mesh)

        # 2. 定义新的 base_link 坐标系的变换矩阵
        T_base_new = np.eye(4)
        T_base_new[:3, :3] = aligned_rotation
        T_base_new[:3, 3] = base_link_original_pos
        T_base_new_inv = np.linalg.inv(T_base_new)

        # 3. 计算 com 和 obb_center 在新的 base_link 坐标系中的相对位置
        com_in_base = T_base_new_inv @ np.append(com_original, 1)
        obb_in_base = T_base_new_inv @ np.append(obb_center_original_pos, 1)
        
        com_origin_in_base_str = " ".join(f"{x:.6f}" for x in com_in_base[:3])
        obb_origin_in_base_str = " ".join(f"{x:.6f}" for x in obb_in_base[:3])

        # 4. 计算 visual/collision 网格的 origin 变换
        visual_transform = T_base_new_inv
        visual_origin_xyz = visual_transform[:3, 3]
        visual_origin_rpy = trimesh.transformations.euler_from_matrix(visual_transform[:3, :3], 'sxyz')
        
        visual_origin_xyz_str = " ".join(f"{x:.6f}" for x in visual_origin_xyz)
        visual_origin_rpy_str = " ".join(f"{x:.6f}" for x in visual_origin_rpy)

        print(f"  -> CoM 相对于 base_link 的位置: {com_origin_in_base_str}")
        print(f"  -> OBB中心 相对于 base_link 的位置: {obb_origin_in_base_str}")

        # 格式化URDF
        urdf_content = URDF_TEMPLATE_BASE_ROOT.format(
            object_name=object_name,
            visual_mesh=VISUAL_MESH_FILENAME,
            collision_mesh=COLLISION_MESH_FILENAME,
            com_origin_in_base_str=com_origin_in_base_str,
            obb_origin_in_base_str=obb_origin_in_base_str,
            visual_origin_xyz_str=visual_origin_xyz_str,
            visual_origin_rpy_str=visual_origin_rpy_str,
        )
        
        # 写入文件
        output_urdf_path = os.path.join(target_dir, f"{object_name}_z_up.urdf")
        try:
            with open(output_urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"  -> 已成功生成URDF文件: {output_urdf_path}")
        except IOError as e:
            print(f"  -> 错误: 无法写入文件。原因: {e}")

if __name__ == "__main__":
    generate_urdf_files_z_up()