import random

# ワールドのヘッダー部分（Heightmap指定を含む）
sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="random_flat_forest">
    <include><uri>model://sun</uri></include>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Gray</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
"""

sdf_body = ""
tree_types = ["model://oak_tree"]

# 配置設定
grid_spacing = 5.0  # 碁盤の目の間隔 (5m)
std_dev = 0.5       # 標準偏差 (50cm = 0.5m)
range_min = -40
range_max = 40

tree_count = 0

# 5m間隔の碁盤の目状にループ生成
for x_grid in range(range_min, range_max + 1, int(grid_spacing)):
    for y_grid in range(range_min, range_max + 1, int(grid_spacing)):
        tree_type = random.choice(tree_types)
        
        # 平均0m、標準偏差0.5m（50cm）の正規分布で中心からランダムな方向にずらす
        dx = random.gauss(0.0, std_dev)
        dy = random.gauss(0.0, std_dev)
        
        x = x_grid + dx
        y = y_grid + dy
        yaw = random.uniform(0, 6.28)

        # 形状（スケール）を微変形させて個体差をつける
        scale_x = random.uniform(0.7, 1.4)
        scale_y = scale_x * random.uniform(0.9, 1.1)
        scale_z = random.uniform(0.8, 1.3)

        sdf_body += f"""
    <include>
      <uri>{tree_type}</uri>
      <name>tree_{tree_count}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.2f}</pose>
      <scale>{scale_x:.2f} {scale_y:.2f} {scale_z:.2f}</scale>
    </include>"""
        tree_count += 1

sdf_footer = """
  </world>
</sdf>
"""

with open("world/flat_kurifarm.world", "w") as f:
    f.write(sdf_header + sdf_body + sdf_footer)

print(f"flat_kurifarm.world を生成しました。（合計 {tree_count} 本の木を配置）")
