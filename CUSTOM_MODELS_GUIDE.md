# Hunter SE Gazebo カスタム3Dモデル完全ガイド
## STL/DAEファイルによるオリジナルワールド構築からSLAM・AtoB走行まで

## 概要
このガイドでは、Hunter SE Gazeboシミュレーション環境でSTL（Stereolithography）およびDAE（Collada）ファイルを使用したオリジナルワールドの構築から、SLAM（同時位置推定地図作成）、そして最終的なAtoB走行（自律ナビゲーション）までの完全なワークフローを説明します。

## ディレクトリ構造

```
hunter_se_gazebo/
├── meshes/
│   ├── stl/           # STLファイル格納ディレクトリ
│   ├── dae/           # DAEファイル格納ディレクトリ
│   └── textures/      # テクスチャファイル格納ディレクトリ
├── models/
│   ├── stl_model_template/    # STL用テンプレート
│   ├── dae_model_template/    # DAE用テンプレート
│   ├── field_stl/             # サンプルSTLモデル
│   └── field_dae/             # サンプルDAEモデル
├── world/
│   └── custom_models.world    # カスタムモデル用ワールドファイル
├── launch/
│   ├── hunter_se_custom_models.launch.py         # 基本表示用launchファイル
│   ├── hunter_se_custom_models_slam.launch.py    # SLAM用launchファイル
│   └── hunter_se_custom_models_navigation.launch.py  # AtoB走行用launchファイル
└── scripts/
    └── create_model_from_mesh.py  # モデル自動生成スクリプト
```

## 使用方法

### 1. 手動でモデルを作成する場合

#### Step 1: メッシュファイルの配置
STLまたはDAEファイルを適切なディレクトリに配置します：
```bash
# STLファイルの場合
cp your_model.stl src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/stl/

# DAEファイルの場合
cp your_model.dae src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/dae/
```

#### Step 2: モデルディレクトリの作成
```bash
mkdir src/ugv_sim/hunter_se/hunter_se_gazebo/models/your_model_name
```

#### Step 3: model.sdfファイルの作成
`models/your_model_name/model.sdf`を作成し、以下の内容を参考にして編集：

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="your_model_name">
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file:///path/to/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/stl/your_model.stl</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
        <material>
          <script>
            <name>Gazebo/Grey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
      
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>file:///path/to/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/stl/your_model.stl</uri>
            <scale>1.0 1.0 1.0</scale>
          </mesh>
        </geometry>
      </collision>
      
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>1.0</iyy>
          <iyz>0.0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>
```

#### Step 4: model.configファイルの作成
`models/your_model_name/model.config`を作成：

```xml
<?xml version="1.0"?>
<model>
  <name>Your Model Name</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  
  <author>
    <name>Your Name</name>
    <email>your@email.com</email>
  </author>
  
  <description>
    Description of your custom model
  </description>
</model>
```

### 2. 自動スクリプトを使用する場合

自動生成スクリプトを使用してより簡単にモデルを作成できます：

```bash
# 基本的な使用方法
python3 src/ugv_sim/hunter_se/hunter_se_gazebo/scripts/create_model_from_mesh.py path/to/your_mesh_file.stl

# オプション付きの使用方法
python3 src/ugv_sim/hunter_se/hunter_se_gazebo/scripts/create_model_from_mesh.py \
    path/to/your_mesh_file.dae \
    --name "my_custom_model" \
    --description "My custom 3D model" \
    --static
```

### 3. ワールドファイルにモデルを追加

`world/custom_models.world`ファイルを編集してモデルを追加：

```xml
<!-- Custom Model -->
<include>
  <uri>model://your_model_name</uri>
  <pose>x y z roll pitch yaw</pose>
</include>
```

### 4. シミュレーションの起動

カスタムモデルを含むシミュレーションを起動：

```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models.launch.py
```

## モデル設定のカスタマイズ

### 物理プロパティの調整

#### Static vs Dynamic
- `<static>true</static>`: 物理エンジンの影響を受けない静的オブジェクト
- `<static>false</static>`: 物理エンジンの影響を受ける動的オブジェクト

#### 質量と慣性モーメント
```xml
<inertial>
  <mass>10.0</mass>  <!-- 質量 (kg) -->
  <inertia>
    <ixx>1.0</ixx>   <!-- x軸周りの慣性モーメント -->
    <iyy>1.0</iyy>   <!-- y軸周りの慣性モーメント -->
    <izz>1.0</izz>   <!-- z軸周りの慣性モーメント -->
  </inertia>
</inertial>
```

#### 摩擦係数
```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>   <!-- 主摩擦係数 -->
      <mu2>1.0</mu2> <!-- 副摩擦係数 -->
    </ode>
  </friction>
</surface>
```

### マテリアルとテクスチャ

#### 基本マテリアル
利用可能なGazebo標準マテリアル：
- `Gazebo/Grey`
- `Gazebo/Wood`
- `Gazebo/Green`
- `Gazebo/Blue`
- `Gazebo/Red`
- `Gazebo/White`
- `Gazebo/Black`

#### カスタムマテリアル
```xml
<material>
  <ambient>0.5 0.5 0.5 1.0</ambient>  <!-- 環境光反射 -->
  <diffuse>0.8 0.8 0.8 1.0</diffuse>  <!-- 拡散反射 -->
  <specular>0.1 0.1 0.1 1.0</specular> <!-- 鏡面反射 -->
</material>
```

### スケーリング
```xml
<geometry>
  <mesh>
    <uri>package://hunter_se_gazebo/meshes/stl/model.stl</uri>
    <scale>2.0 2.0 2.0</scale>  <!-- X, Y, Z方向のスケール -->
  </mesh>
</geometry>
```

## トラブルシューティング

### よくある問題

1. **モデルが表示されない**
   - メッシュファイルのパスが正しいか確認
   - URIが`file://`絶対パス形式になっているか確認
   - 例: `file:///home/user/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/stl/model.stl`

2. **ロボットがオブジェクトに重なってスタックする**
   - スポーン位置を調整: `start_x:=-2 start_y:=0 start_z:=0.3`
   - ワールド内のオブジェクト配置を確認
   - カスタムスポーン位置での起動を試行

3. **package://URIエラーが発生する**
   - `package://`から`file://`絶対パスに変更
   - GAZEBO_MODEL_PATH環境変数が正しく設定されているか確認

4. **モデルが期待した場所に表示されない**
   - `<pose>`タグの値を確認
   - 座標系の理解（Gazeboは右手座標系）

5. **テクスチャが適用されない**
   - DAEファイルの場合、テクスチャファイルが`meshes/textures/`にあるか確認
   - マテリアル設定が正しいか確認

6. **物理演算が期待通りに動作しない**
   - 慣性モーメントの値が適切か確認
   - 衝突判定のメッシュが複雑すぎる場合は簡略化を検討

7. **SLAMが正常に動作しない**
   - ロボットの初期位置がオブジェクトと重ならないよう調整
   - LiDARセンサーの動作確認
   - nav2パラメータファイルの設定確認

### デバッグのヒント

- Gazeboのコンソール出力でエラーメッセージを確認
- RVizの「TF」や「Robot Model」表示でモデルの読み込み状況を確認
- `gztopic list`でトピック一覧を確認

## サンプルファイル

プロジェクトにはサンプルとして`field.stl`と`field.dae`が含まれています。これらのファイルを参考にして独自のモデルを作成してください。

## 完全ワークフロー：オリジナルワールドからAtoB走行まで

### Phase 1: カスタムモデル環境の構築

#### 1.1 メッシュファイルの準備と配置
```bash
# STL/DAEファイルをワークスペースの適切な場所に配置
cp your_model.stl src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/stl/
cp your_model.dae src/ugv_sim/hunter_se/hunter_se_gazebo/meshes/dae/
```

#### 1.2 パッケージのビルド
```bash
cd ~/hunter_ws
colcon build --packages-select hunter_se_gazebo
source install/setup.bash
```

#### 1.3 カスタムモデル環境の表示確認
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models.launch.py
```

### Phase 2: SLAM（地図作成）

#### 2.1 SLAM環境の起動
```bash
cd ~/hunter_ws
source install/setup.bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_slam.launch.py
```

#### 2.2 SLAMによる地図作成
1. **Gazebo起動後の確認**
   - Gazeboでカスタムモデル環境が表示される
   - RVizでロボットの周辺が表示される

2. **地図作成プロセス**
   - RVizの「2D Nav Goal」ツールを使用してロボットを移動
   - ロボットが移動するにつれて地図が徐々に作成される
   - LiDARスキャンで環境をマッピング

3. **地図作成完了の判断基準**
   - 移動予定エリア全体がマップされている
   - 壁や障害物の輪郭が明確に表示されている
   - グレーエリア（未探索領域）が十分少ない

#### 2.3 地図の保存
```bash
# 地図作成完了後、新しいターミナルで実行
ros2 run nav2_map_server map_saver_cli -f your_map_name
```

作成される地図ファイル：
- `your_map_name.pgm` (地図画像)
- `your_map_name.yaml` (地図メタデータ)

### Phase 3: AtoB走行（ナビゲーション）

#### 3.1 地図ファイルの配置
```bash
# 作成した地図をmapsディレクトリに移動
cp your_map_name.* src/ugv_sim/hunter_se/hunter_se_gazebo/maps/
```

#### 3.2 ナビゲーション環境の起動
```bash
cd ~/hunter_ws
colcon build --packages-select hunter_se_gazebo
source install/setup.bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py
```

#### 3.3 AtoB走行の実行
1. **初期化確認**
   - Gazeboでカスタムモデル環境が表示される
   - RVizで作成した地図が読み込まれる
   - ロボットの現在位置が地図上に表示される

2. **初期位置の調整（必要な場合）**
   - RVizの「2D Pose Estimate」ツールを使用
   - ロボットの実際の位置をクリック・ドラッグで設定
   - パーティクルフィルターが収束するまで待機

3. **目標地点の設定とナビゲーション実行**
   - RVizの「2D Nav Goal」ツールを選択
   - 目標地点をクリック・ドラッグで指定
   - ロボットが自動的に経路計画し移動開始
   - 障害物回避しながら目標地点まで自律移動

## 利用可能なLaunchファイル

### 基本表示モード
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models.launch.py
```
- カスタムモデル環境の表示のみ
- ロボットの手動操縦が可能

### SLAMモード
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_slam.launch.py
```
- SLAM機能有効
- リアルタイム地図作成
- ロボット移動による環境マッピング

### ナビゲーションモード
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py
```
- 作成済み地図を使用した自律ナビゲーション
- AtoB走行機能
- 経路計画と障害物回避

## 実践的な使用例

### シナリオ1: 新環境での地図作成から運用まで
1. STL/DAEファイルで環境モデル作成
2. SLAMモードで地図作成（10-15分）
3. 地図保存後、ナビゲーションモードで自律走行テスト

### シナリオ2: 既存地図を使った運用テスト
1. 既存の地図ファイル（your_map_name.*）をmapsディレクトリに配置
2. ナビゲーションモード直接起動
3. 複数の目標地点での走行テスト実施

## 高度な設定

### カスタムマップファイルの使用
特定の地図ファイルを使用したい場合：
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py map:=/path/to/your/custom_map.yaml
```

### 起動パラメータのカスタマイズ
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py \
    start_x:=5.0 start_y:=3.0 start_yaw:=1.57
```

### ロボットスポーン位置の調整

**デフォルトスポーン位置:** `x=-2, y=0, z=0.3`

この位置は以下の理由で選択されています：
- fieldオブジェクト（原点配置）との衝突回避
- 地面から適切な高さでの安全なスポーン
- SLAM開始時の良好な視界確保

**スポーン位置が不適切な場合の対処法：**
```bash
# カスタムスポーン位置での起動例
ros2 launch hunter_se_gazebo hunter_se_custom_models_slam.launch.py \
    start_x:=-5.0 start_y:=2.0 start_z:=0.5
```

## 既存のNav2機能との併用

このカスタムモデル機能は既存のNav2ナビゲーション機能と完全に統合されています：
- ✅ 地図ベースの自己位置推定
- ✅ 動的経路計画
- ✅ 障害物回避
- ✅ リカバリー行動
- ✅ コストマップベースのナビゲーション