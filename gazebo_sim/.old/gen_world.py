import random

# ワールドのヘッダー部分（Heightmap指定を含む）
sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="random_forest">
    <include><uri>model://sun</uri></include>
    <model name="heightmap">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://terrain.png</uri>
              <size>100 100 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>
                <normal>file://media/materials/textures/flat_normal.png</normal>
                <size>1</size>
              </texture>
              <uri>file://terrain.png</uri>
              <size>100 100 10</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>
"""

sdf_body = ""
tree_types = ["model://oak_tree", "model://pine_tree"]
#tree_types = ["model://Oak tree", "model://Pine tree"]
#tree_types = ["https://fuel.gazebo.org/1.0/openrobotics/models/Oak tree",
#    "https://fuel.gazebo.org/1.0/openrobotics/models/Pine tree"]

# 50本の木をランダム生成
for i in range(50):
    tree_type = random.choice(tree_types)
    x = random.uniform(-40, 40)
    y = random.uniform(-40, 40)
    yaw = random.uniform(0, 6.28)
    
    # 形状（スケール）を微変形させて個体差をつける
    scale_x = random.uniform(0.7, 1.4)
    scale_y = scale_x * random.uniform(0.9, 1.1)
    scale_z = random.uniform(0.8, 1.3)

    sdf_body += f"""
    <include>
      <uri>{tree_type}</uri>
      <name>tree_{i}</name>
      <pose>{x:.2f} {y:.2f} 0 0 0 {yaw:.2f}</pose>
      <scale>{scale_x:.2f} {scale_y:.2f} {scale_z:.2f}</scale>
    </include>"""

sdf_footer = """
  </world>
</sdf>
"""

with open("world/random_forest.world", "w") as f:
    f.write(sdf_header + sdf_body + sdf_footer)

print("random_forest.world を生成しました。")
