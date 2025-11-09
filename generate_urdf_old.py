import os

# --- 配置区域 ---
# 1. 设置您的YCB数据集的根目录路径
BASE_YCB_PATH = "ycb_assets"  # <-- !!! 修改这里 !!!

# 2. 在这里便捷地修改用于visual和collision的网格文件名
#    这些是YCB数据集中常见的一些选项：
#    Visual Mesh: "textured.obj", "textured.dae"
#    Collision Mesh: "nontextured.stl", "nontextured.ply", "textured_vhacd.obj" (凸分解模型)
VISUAL_MESH_FILENAME = "textured.obj"
COLLISION_MESH_FILENAME = "textured_vhacd.obj" # 推荐使用凸分解模型以提高仿真性能
# --- 配置结束 ---


# URDF模板，使用f-string格式化
URDF_TEMPLATE = """<?xml version="1.0"?>
<robot name="{object_name}">
  <link name="{object_name}">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/> 
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{visual_mesh}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="{collision_mesh}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""

def generate_urdf_files():
    """
    遍历YCB数据集目录并为每个对象生成URDF文件。
    """
    print("--- 开始生成URDF文件 ---")
    if not os.path.isdir(BASE_YCB_PATH):
        print(f"错误: 目录 '{BASE_YCB_PATH}' 不存在。请检查配置。")
        return

    # 获取所有对象目录
    try:
        object_folders = [d for d in os.listdir(BASE_YCB_PATH) if os.path.isdir(os.path.join(BASE_YCB_PATH, d))]
    except FileNotFoundError:
        print(f"错误: 无法访问目录 '{BASE_YCB_PATH}'。")
        return

    generated_count = 0
    for folder_name in sorted(object_folders):
        # 简单的检查，确保这是一个YCB对象文件夹 (e.g., '002_master_chef_can')
        if not folder_name[:3].isdigit() or '_' not in folder_name:
            continue
            
        object_name = folder_name[:]  # 从 '002_master_chef_can' 提取 'master_chef_can'
        
        # 构建目标路径
        target_dir = os.path.join(BASE_YCB_PATH, folder_name, "google_16k")
        if not os.path.isdir(target_dir):
            print(f"警告: 在 '{folder_name}' 中未找到 'google_16k' 目录，已跳过。")
            continue

        print(f"正在处理对象: {object_name}")

        # 格式化URDF内容
        urdf_content = URDF_TEMPLATE.format(
            object_name=object_name,
            visual_mesh=VISUAL_MESH_FILENAME,
            collision_mesh=COLLISION_MESH_FILENAME
        )

        # 定义输出文件名
        output_urdf_path = os.path.join(target_dir, f"{object_name}.urdf")

        # 写入文件
        try:
            with open(output_urdf_path, 'w') as f:
                f.write(urdf_content)
            print(f"  -> 已成功生成URDF文件: {output_urdf_path}")
            generated_count += 1
        except IOError as e:
            print(f"  -> 错误: 无法写入文件 {output_urdf_path}。原因: {e}")

    print("\n--- 操作完成 ---")
    print(f"总共为 {generated_count} 个对象生成了URDF文件。")

if __name__ == "__main__":
    generate_urdf_files()