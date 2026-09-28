# Gazeboのランダムマップ作製
## 1. 木のモデルを生成
Blenderが必要です。事前にダウンロードしてください。  
既存スクリプトでは５本のランダムな木が生成されます。  
```bash
blender --background --python kuritree.py
```

## 2. マップ生成
1.で生成した木を使ったマップを作成します。
```bash
python gen_flat_kurifarm.py
```
現時点では地面は平面です。将来的に起伏を作る予定。

----
```bash
./scripts/launch-docker.sh --target=sim
```
でDocker起動後、当ディレクトリで
```bash
./launch-flat-kurifarm.sh
```
で生成したマップを読み込みGazeboが起動します。