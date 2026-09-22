import bpy

if "add_curve_sapling" not in bpy.context.preferences.addons:
    bpy.ops.preferences.addon_enable(module="add_curve_sapling")
    print("addon added")

# オブジェクトモードであることを確認
if bpy.context.mode != 'OBJECT':
    bpy.ops.object.mode_set(mode='OBJECT')
    print("set to object mode")

# Sapling Tree Genの実行
bpy.ops.curve.tree_add(
    do_update=True,
    # 幹・枝の設定
    levels=4,           # 枝分かれの段階（レベル）
    bevel=True,          # メッシュ化の厚みをつける
    prune=False,         # 剪定（剪定を行わない）
    
    # 葉の設定
    showLeaves=True,     # 葉を表示する
    leafShape='hex',     # 葉の形状 ('hex', 'rect', 'dodeca' など)
    leaves=0,           # 枝あたりの葉の枚数
    leafScale=0.15,      # 葉のサイズ
    
    # 乱数シード値
    seed=42,             # ランダムシード（数値を変えると木のかたちが変わる）
    
    length=(1.0, 0.8, 0.6, 0.5),
    splitAngle=(45.0, 45.0, 45.0, 0.0),
    outwardAttraction=(0, 0.5, 0.5, 0)
)
