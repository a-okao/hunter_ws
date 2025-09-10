# カスタム3Dモデル（STL/DAE）読み込みガイド

## 概要
このガイドでは、Hunter SE Gazeboシミュレーション環境でSTL（Stereolithography）およびDAE（Collada）ファイルを使用してオリジナルの3Dモデルを読み込む方法について説明します。

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
│   └── hunter_se_custom_models.launch.py  # 起動用launchファイル
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
            <uri>package://hunter_se_gazebo/meshes/stl/your_model.stl</uri>
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
            <uri>package://hunter_se_gazebo/meshes/stl/your_model.stl</uri>
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
   - URIが`package://hunter_se_gazebo/...`形式になっているか確認

2. **モデルが期待した場所に表示されない**
   - `<pose>`タグの値を確認
   - 座標系の理解（Gazeboは右手座標系）

3. **テクスチャが適用されない**
   - DAEファイルの場合、テクスチャファイルが`meshes/textures/`にあるか確認
   - マテリアル設定が正しいか確認

4. **物理演算が期待通りに動作しない**
   - 慣性モーメントの値が適切か確認
   - 衝突判定のメッシュが複雑すぎる場合は簡略化を検討

### デバッグのヒント

- Gazeboのコンソール出力でエラーメッセージを確認
- RVizの「TF」や「Robot Model」表示でモデルの読み込み状況を確認
- `gztopic list`でトピック一覧を確認

## サンプルファイル

プロジェクトにはサンプルとして`field.stl`と`field.dae`が含まれています。これらのファイルを参考にして独自のモデルを作成してください。

## 既存のNav2機能との併用

このカスタムモデル機能は既存のNav2ナビゲーション機能と併用できます。カスタムモデルを配置した環境でもナビゲーションが正常に動作するよう設計されています。

ナビゲーション機能を使用する場合は、適切なマップファイルも合わせて準備してください。