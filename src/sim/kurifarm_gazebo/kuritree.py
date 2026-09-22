import bpy
import random
import os
import sys

def create_tree_model(seed):
	bpy.ops.curve.tree_add(
		do_update=True,
		chooseSet='0',
		bevel=True,
		prune=False,
		showLeaves=False,
		useArm=False,
		seed=seed,
		handleType='0',
		levels=2,
		length=(1.22, 0.3, 0.4, 0.1),
		lengthV=(0.36, 0.1, 0.93, 0),
		taperCrown=0.56,
		branches=(5, 9, 8, 2),
		curveRes=(10, 9, 6, 1),
		curve=(-42.54, -34.41, 0, 0),
		curveV=(30.27, 31.41, 31.41, 0),
		curveBack=(0.0600002, 0.0300031, 0.0899992, 0),
		baseSplits=4,
		segSplits=(0.21, 0.32, 0.75, 0),
		splitByLen=True,
		rMode='rotate',
		splitAngle=(46.62, 46.89, 27.28, 0),
		splitAngleV=(0.14, 1.1, 0.98, 0),
		scale=5,
		scaleV=2,
		attractUp=(0.47, 0.260156, 0.39, 0),
		attractOut=(0.6, 0.31, 0.34, 0),
		shape='7',
		shapeS='10',
		customShape=(0.5, 1, 0.3, 0.5),
		branchDist=1.5,
		nrings=0,
		baseSize=0.06,
		baseSize_s=0.93,
		splitHeight=0.36,
		splitBias=0.58,
		ratio=0.195,
		minRadius=0.01,
		closeTip=False,
		rootFlare=1,
		autoTaper=True,
		taper=(1, 1, 1, 1),
		radiusTweak=(1, 1, 1, 1),
		ratioPower=0.81,
		downAngle=(14.55, 0.589998, 4.56, 24.81),
		downAngleV=(0.84, 7.69, 7.3, 10),
		useOldDownAngle=True,
		useParentAngle=True,
		rotate=(135, 135, 135, 0),
		rotateV=(0.929997, 0, 0, 0),
		scale0=0.16,
		scaleV0=0.27,
		pruneWidth=0.34,
		pruneBase=0.12,
		pruneWidthPeak=0.5,
		prunePowerHigh=0.5,
		prunePowerLow=0.001,
		pruneRatio=0.75,
		leaves=33,
		leafDownAngle=30,
		leafDownAngleV=-10,
		leafRotate=137.5, leafRotateV=15, leafScale=0.4, leafScaleX=0.2, leafScaleT=0.1, leafScaleV=0.15, leafShape='hex', bend=0, leafangle=-12, horzLeaves=True, leafDist='1', bevelRes=1, resU=4, armAnim=False, previewArm=False, leafAnim=False, frameRate=1, loopFrames=0, wind=1, gust=1, gustF=0.075, af1=1, af2=1, af3=4, makeMesh=False, armLevels=2, boneStep=(1, 1, 1, 1)
		)

def clear_scene():
    # 全オブジェクトを選択して削除
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    
    # 未使用のデータブロック（メッシュやカーブなど）をクリーンアップ
	# データブロックはオブジェクトの子。オブジェクトを残してもメモリ上に残ることがあるらしい
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in bpy.data.curves:
        if block.users == 0:
            bpy.data.curves.remove(block)

def main():
	if "add_curve_sapling" not in bpy.context.preferences.addons:
		bpy.ops.preferences.addon_enable(module="add_curve_sapling")
		print("addon added")
	bpy.context.view_layer.update()

	gazebo_model_dir = f"{os.path.dirname(__file__)}/model"
	os.makedirs(gazebo_model_dir, exist_ok=True)

	for i in range(5):
		model_name = f"random_tree_{i}"
		model_dir = os.path.join(gazebo_model_dir, model_name)
		mesh_dir = os.path.join(model_dir, "meshes")

		os.makedirs(mesh_dir, exist_ok=True)

		# シーン内オブジェクトを削除
		clear_scene()

		create_tree_model(random.randint(0, 1e5))

		bpy.ops.object.select_all(action="SELECT")
		if bpy.context.selected_objects:
			bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
		bpy.ops.object.convert(target="MESH")

		# メッシュを統合
		mesh_objs = [o for o in bpy.context.selected_objects if o.type == 'MESH']

		if mesh_objs:
			# 統合の基準となるアクティブオブジェクトを設定
			bpy.context.view_layer.objects.active = mesh_objs[0]
			bpy.ops.object.join()
			
			tree_obj = bpy.context.active_object

			# 5. 統合された単一メッシュに対して Decimate を適用
			mod = tree_obj.modifiers.new(name="Decimate", type='DECIMATE')
			mod.ratio = DECIMATE_RATIO
			bpy.ops.object.modifier_apply(modifier=mod.name)

		dae_path = os.path.join(mesh_dir, f"{model_name}.dae")
		bpy.ops.object.select_all(action="SELECT")
		bpy.ops.wm.collada_export(filepath=dae_path, check_existing=False)


		# 1. model.config の生成
		config_content = f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>Auto Generated</name></author>
  <description>Auto generated random tree</description>
</model>"""

		with open(os.path.join(model_dir, "model.config"), "w") as f:
			f.write(config_content)

    	# 2. model.sdf の生成
		sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{model_name}.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>model://{model_name}/meshes/{model_name}.dae</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>"""

		with open(os.path.join(model_dir, "model.sdf"), "w") as f:
			f.write(sdf_content)

		print(f"Exported: {model_name}")

if __name__ == "__main__":
	# 削減率の設定 (0.1 = ポリゴン数を約10%に削減)
	DECIMATE_RATIO = 0.15
	main()
