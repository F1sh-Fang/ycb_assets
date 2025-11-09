import os
import trimesh
import numpy as np

# --- 配置区域 ---
# 1. 设置您的YCB数据集的根目录路径
BASE_YCB_PATH = "ycb_assets"  # <-- !!! 修改这里 !!!

# 2. 网格文件名
VISUAL_MESH_FILENAME = "textured.obj"
COLLISION_MESH_FILENAME = "textured_vhacd.obj"
# --- 配置结束 ---


# URDF模板
# 1. base_link 是 XY 投影中心。visual 和 collision 网格只应用 x 和 y 的偏移。
# 2. center_of_mass link 代表真正的3D质心，通过一个 fixed joint 连接到 base_link。
#    这个 joint 的 origin 的 Z 值就是质心的 z 坐标。
URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="{object_name}">
  <!-- base_link 的原点位于模型在Z轴上的投影中心 -->
  <link name="base_link">
    <inertial>
      <!-- 惯性参考系的原点是相对于 link 原点的。
           由于 link 原点在 (cx, cy, 0)，而质心在 (cx, cy, cz)，
           因此惯性原点需要向上平移 cz。 -->
      <origin xyz="0 0 {center_z}" rpy="0 0 0"/>
      <mass value="0.1"/> 
      <!-- TODO: 更精确的惯性张量可以从 trimesh.mesh.moment_inertia 计算 -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <!-- 将模型的几何中心在X和Y方向上对齐到 link 的原点 -->
      <origin xyz="{xy_offset_str}" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{visual_mesh}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="{xy_offset_str}" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{collision_mesh}"/>
      </geometry>
    </collision>
  </link>

  <!-- 这个 link 代表了模型真正的3D几何中心 (质心) -->
  <link name="center_of_mass"/>

  <joint name="base_to_com" type="fixed">
    <parent link="base_link"/>
    <child link="center_of_mass"/>
    <!-- 这个 joint 将 center_of_mass link 放置在相对于 base_link 的 (0, 0, cz) 位置 -->
    <origin xyz="0 0 {center_z}" rpy="0 0 0"/>
  </joint>

</robot>
"""

def get_mesh_center(mesh_path):
    """
    使用 trimesh 加载模型并计算其质心 (centroid)。
    返回一个表示质心坐标 [cx, cy, cz] 的numpy数组。
    """
    try:
        # 使用 process=False 确保 trimesh 不会自动居中模型
        mesh = trimesh.load(mesh_path, force='mesh', process=False)
        return mesh.centroid
    except Exception as e:
        print(f"  -> 错误: 使用trimesh加载或处理 '{mesh_path}' 时出错: {e}")
        return None

def generate_urdf_files():
    """
    遍历YCB数据集目录，计算每个碰撞模型的中心，并生成中心对齐的URDF文件。
    """
    print("--- 开始生成URDF文件 ---")
    if not os.path.isdir(BASE_YCB_PATH):
        print(f"错误: 目录 '{BASE_YCB_PATH}' 不存在。请检查配置。")
        return

    try:
        object_folders = [d for d in os.listdir(BASE_YCB_PATH) if os.path.isdir(os.path.join(BASE_YCB_PATH, d))]
    except FileNotFoundError:
        print(f"错误: 无法访问目录 '{BASE_YCB_PATH}'。")
        return

    generated_count = 0
    for folder_name in sorted(object_folders):
        if not folder_name[:3].isdigit() or '_' not in folder_name:
            continue
            
        object_name = folder_name
        
        target_dir = os.path.join(BASE_YCB_PATH, folder_name, "google_16k")
        if not os.path.isdir(target_dir):
            print(f"警告: 在 '{folder_name}' 中未找到 'google_16k' 目录，已跳过。")
            continue

        print(f"正在处理对象: {object_name}")

        # 计算碰撞模型的3D质心
        collision_mesh_path = os.path.join(target_dir, COLLISION_MESH_FILENAME)
        if not os.path.exists(collision_mesh_path):
            print(f"  -> 警告: 碰撞模型 '{COLLISION_MESH_FILENAME}' 不存在，已跳过。")
            continue
        
        center_offset = get_mesh_center(collision_mesh_path)
        
        if center_offset is None:
            print(f"  -> 无法计算中心，已跳过 {object_name}。")
            continue

        # 1. 计算用于 visual/collision 的XY平面偏移向量 (-cx, -cy, 0)
        xy_offset_vector = np.array([-center_offset[0], -center_offset[1], 0.0])
        xy_offset_str = " ".join(f"{x:.6f}" for x in xy_offset_vector) # 使用格式化确保精度

        # 2. 获取质心的Z坐标，用于 inertial 和 joint 的 origin
        center_z_str = f"{center_offset[2]:.6f}"
        
        print(f"  -> 3D 质心 (cx, cy, cz): [{center_offset[0]:.4f}, {center_offset[1]:.4f}, {center_offset[2]:.4f}]")
        print(f"  -> 应用于模型的XY偏移: {xy_offset_str}")
        print(f"  -> 质心的Z坐标 (cz): {center_z_str}")

        # 格式化URDF内容
        urdf_content = URDF_TEMPLATE.format(
            object_name=object_name,
            visual_mesh=VISUAL_MESH_FILENAME,
            collision_mesh=COLLISION_MESH_FILENAME,
            xy_offset_str=xy_offset_str,
            center_z=center_z_str
        )

        # 定义输出文件名
        output_urdf_path = os.path.join(target_dir, f"{object_name}.urdf")

        # 写入文件
        try:
            with open(output_urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"  -> 已成功生成双link URDF文件: {output_urdf_path}")
            generated_count += 1
        except IOError as e:
            print(f"  -> 错误: 无法写入文件 {output_urdf_path}。原因: {e}")

    print("\n--- 操作完成 ---")
    print(f"总共为 {generated_count} 个对象生成了URDF文件。")

if __name__ == "__main__":
    generate_urdf_files()