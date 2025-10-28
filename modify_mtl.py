import os

# --- 配置区域 ---
# 1. 设置您的YCB数据集的根目录路径
BASE_YCB_PATH = "ycb_assets" 
# --- 配置结束 ---


# 要写入新的 textured.mtl 文件中的内容
NEW_MTL_CONTENT = """newmtl material_0
Ka 0.200000 0.200000 0.200000
Kd 0.639216 0.639216 0.639216
Ks 1.000000 1.000000 1.000000
Tr 1.000000
illum 2
Ns 0.000000
map_Kd texture_map.png
"""

def modify_mtl_files():
    """
    遍历YCB数据集，备份并重写 textured.mtl 文件。
    """
    print("--- 开始修改MTL文件 ---")
    if not os.path.isdir(BASE_YCB_PATH):
        print(f"错误: 目录 '{BASE_YCB_PATH}' 不存在。请检查配置。")
        return

    try:
        object_folders = [d for d in os.listdir(BASE_YCB_PATH) if os.path.isdir(os.path.join(BASE_YCB_PATH, d))]
    except FileNotFoundError:
        print(f"错误: 无法访问目录 '{BASE_YCB_PATH}'。")
        return

    modified_count = 0
    for folder_name in sorted(object_folders):
        if not folder_name[:3].isdigit() or '_' not in folder_name:
            continue
            
        object_name = folder_name[4:]
        
        target_dir = os.path.join(BASE_YCB_PATH, folder_name, "google_16k")
        if not os.path.isdir(target_dir):
            continue

        mtl_path = os.path.join(target_dir, "textured.mtl")
        backup_path = os.path.join(target_dir, "textured.mtl.bak")

        if os.path.exists(mtl_path):
            print(f"正在处理对象: {object_name}")
            
            # 1. 备份原始文件（如果备份不存在）
            if not os.path.exists(backup_path):
                try:
                    os.rename(mtl_path, backup_path)
                    print(f"  -> 已备份原始文件到: {backup_path}")
                except OSError as e:
                    print(f"  -> 错误: 无法备份文件。原因: {e}")
                    continue
            else:
                print(f"  -> 备份文件已存在，跳过备份步骤。")

            # 2. 写入新文件
            try:
                with open(mtl_path, 'w') as f:
                    f.write(NEW_MTL_CONTENT)
                print(f"  -> 已创建新的MTL文件: {mtl_path}")
                modified_count += 1
            except IOError as e:
                print(f"  -> 错误: 无法写入新文件。原因: {e}")
    
    print("\n--- 操作完成 ---")
    print(f"总共为 {modified_count} 个对象修改了MTL文件。")


if __name__ == "__main__":
    modify_mtl_files()