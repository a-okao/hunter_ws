# Hunter SE SLAM & 自律走行システム

Hunter SEロボットを使用したSLAM（Simultaneous Localization and Mapping）と自律走行システムのマニュアルです。

## システム概要

- **ロボット**: Hunter SE（4輪独立ステアリング）
- **センサー**: MID-360 LiDAR
- **環境**: ROS2 Humble + Gazebo + Nav2
- **機能**: 
  - SLAM（同時位置推定地図作成）
  - 自律ナビゲーション・AtoB走行
  - マップ作成・保存
  - **カスタム3Dモデル環境（STL/DAE対応）**
  - オリジナルワールド構築からSLAM・ナビゲーションまでの完全ワークフロー

## 目次

1. [セットアップ手順](#セットアップ手順)
2. [パッケージ構成](#パッケージ構成)
3. [基本的なSLAMとナビゲーション](#基本的なslamとナビゲーション)
4. [カスタム3Dモデル環境](#カスタム3dモデル環境)
5. [トラブルシューティング](#トラブルシューティング)
6. [パラメータ調整](#パラメータ調整)

## セットアップ手順

### 1. 前提条件

以下がインストールされていることを確認してください：
- Ubuntu 22.04 LTS
- ROS2 Humble

### 2. リポジトリのクローン

```bash
# ホームディレクトリに移動
cd ~

# リポジトリをクローン
git clone https://github.com/a-okao/hunter_ws.git

# ワークスペースディレクトリに移動
cd hunter_ws
```

### 3. 必要な依存関係のインストール

```bash
# システムパッケージの更新
sudo apt update

# ROS2ナビゲーション関連パッケージのインストール
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-gazebo-ros-pkgs
```

### 4. ワークスペースのビルド

```bash
# ワークスペースのビルド
colcon build --packages-select hunter_se_description hunter_se_gazebo

# 環境変数の設定
source install/setup.bash

# 自動的に環境変数を設定（オプション）
echo "source ~/hunter_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. セットアップの確認

```bash
# パッケージが正しく認識されているか確認
ros2 pkg list | grep hunter_se

# 以下が出力されることを確認：
# hunter_se_description
# hunter_se_gazebo
```

## パッケージ構成

```
hunter_ws/
├── src/ugv_sim/hunter_se/
│   ├── hunter_se_description/          # ロボットモデル定義
│   │   ├── urdf/hunter_se_description.xacro  # URDF定義
│   │   └── rviz/                       # RViz設定ファイル
│   └── hunter_se_gazebo/               # Gazeboシミュレーション
│       ├── world/                      # 環境ファイル
│       │   ├── factory.world           # 工場環境
│       │   └── house.world             # 住宅環境
│       ├── config/nav2/                # ナビゲーション設定
│       │   ├── nav2_params.yaml        # Nav2パラメータ
│       │   └── SLAM_PARAMETER_TUNING_GUIDE.md  # パラメータ調整ガイド
│       ├── launch/                     # 起動ファイル
│       └── maps/                       # 保存されたマップ
└── CLAUDE.md                          # AI運用原則
```

## 基本的なSLAMとナビゲーション

### SLAMによるマップ作成

### 1. SLAMシステムの起動

工場環境でSLAMを開始：

```bash
ros2 launch hunter_se_gazebo hunter_se_simulation_nav2.launch.py
```

### 2. 手動操作でマッピング

別ターミナルでロボットを手動操作：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**操作方法:**
- `i`: 前進
- `k`: 停止
- `j`: 左旋回
- `l`: 右旋回
- `u`,`o`,`,`,`.`: 斜め移動

### 3. マッピング状況の確認

RViz2画面で以下を確認：
- **灰色エリア**: 走行済みの自由空間
- **黒色エリア**: 障害物
- **白色エリア**: 未探索エリア

### 4. マップの保存

マッピング完了後、マップを保存：

```bash
ros2 run nav2_map_server map_saver_cli -f /home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/maps/factory_map
```

### 自律ナビゲーション

### 1. ナビゲーションシステムの起動

保存済みマップを使用した自律ナビゲーション：

```bash
ros2 launch hunter_se_gazebo hunter_se_navigation_test.launch.py map:=/home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/maps/factory_map.yaml
```

### 2. 初期位置の設定

RViz2で「2D Pose Estimate」ツールを使用：
1. ツールバーから「2D Pose Estimate」をクリック
2. マップ上でロボットの実際の位置をクリック
3. ドラッグしてロボットの向きを設定

### 3. 目標地点の設定

RViz2で「2D Nav Goal」ツールを使用：
1. ツールバーから「2D Nav Goal」をクリック
2. 目標地点をクリック
3. ドラッグして目標方向を設定

### 4. 自律走行の確認

- **緑の線**: グローバルプラン（全体経路）
- **黄色の線**: ローカルプラン（局所経路）
- **赤い点群**: LiDARデータ
- **青い雲**: パーティクルフィルタ（位置推定）

## トラブルシューティング

### マップが作成されない場合

**症状**: ロボットを動かしても灰色エリアが出現しない

**解決方法**:
```bash
# TF変換の確認
ros2 run tf2_tools view_frames

# センサーデータの確認
ros2 topic echo /scan --once

# ノードの状態確認
ros2 node list | grep slam
```

### ナビゲーションが動作しない場合

**症状**: 目標設定してもロボットが動かない

**解決方法**:
```bash
# ナビゲーション状態の確認
ros2 topic list | grep nav

# AMCLの位置推定確認
ros2 topic echo /amcl_pose --once

# パスプランニング確認
ros2 topic echo /plan --once
```

### 位置推定がずれる場合

**症状**: ロボットの位置がマップ上でずれている

**解決方法**:
1. 「2D Pose Estimate」で位置を再設定
2. 一度ロボットを回転させて位置推定を安定化
3. AMCLパラメータの調整を検討

## パラメータ調整

### SLAMパラメータの調整

詳細なパラメータ調整ガイドは以下を参照：
```
/home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/config/nav2/SLAM_PARAMETER_TUNING_GUIDE.md
```

### 主要な調整項目

| 用途 | パラメータ | 調整方向 |
|------|------------|----------|
| 高精度マッピング | `resolution` | 0.02に減少 |
| 軽量化 | `throttle_scans` | 2-3に増加 |
| ループクロージャ強化 | `loop_search_maximum_distance` | 5.0に増加 |
| 動的環境対応 | `minimum_time_interval` | 0.2に減少 |

### パラメータ変更手順

1. **設定ファイル編集**:
   ```bash
   nano /home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/config/nav2/nav2_params.yaml
   ```

2. **再ビルド**:
   ```bash
   cd /home/iskhas/hunter_ws
   colcon build --packages-select hunter_se_gazebo
   source install/setup.bash
   ```

3. **動作確認**:
   ```bash
   ros2 launch hunter_se_gazebo hunter_se_simulation_nav2.launch.py
   ```

## カスタム3Dモデル環境

### 概要

STL/DAEファイルを使用してオリジナルの3D環境を構築し、その環境でSLAMとナビゲーションを実行できます。

### 利用可能な起動モード

1. **基本表示モード** - カスタムモデル環境の確認
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models.launch.py
```

2. **SLAMモード** - カスタム環境での地図作成
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_slam.launch.py
```

3. **ナビゲーションモード** - 作成した地図での自律走行
```bash
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py
```

### 基本ワークフロー

1. **STL/DAEファイルをmeshesディレクトリに配置**
2. **SLAMモードで地図作成**
3. **地図保存**: `ros2 run nav2_map_server map_saver_cli -f your_map_name`
4. **ナビゲーションモードで自律走行テスト**

### 詳細なガイド

カスタム3Dモデル環境の詳細な構築手順、設定方法、トラブルシューティングについては、以下の専用ガイドを参照してください：

📖 **[CUSTOM_MODELS_GUIDE.md](./CUSTOM_MODELS_GUIDE.md)**

このガイドには以下の内容が含まれています：
- STL/DAEファイルの配置方法
- 自動モデル生成スクリプトの使用方法
- SLAMからナビゲーションまでの完全ワークフロー
- 起動パラメータのカスタマイズ方法（スポーン位置調整など）
- よくある問題のトラブルシューティング

## 利用可能な環境

### 1. 工場環境（factory.world）
- 30m×30mの工場フロア
- 製造機械、支柱、静的障害物
- 複雑な通路とコーナー

### 2. 住宅環境（house.world）
- 基本的な住宅レイアウト
- シンプルな構造

### 3. カスタム3Dモデル環境（custom_models.world）
- STL/DAEファイルから構築したオリジナル環境
- 完全にカスタマイズ可能な3D空間
- SLAM・ナビゲーション完全対応

## よく使用するコマンド集

```bash
# システム起動
ros2 launch hunter_se_gazebo hunter_se_simulation_nav2.launch.py

# ナビゲーションテスト
ros2 launch hunter_se_gazebo hunter_se_navigation_test.launch.py

# 手動操作
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# マップ保存（基本環境）
ros2 run nav2_map_server map_saver_cli -f ~/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/maps/my_map

# カスタムモデル環境起動
ros2 launch hunter_se_gazebo hunter_se_custom_models_slam.launch.py
ros2 launch hunter_se_gazebo hunter_se_custom_models_navigation.launch.py

# トピック一覧
ros2 topic list

# ノード一覧  
ros2 node list

# パラメータ確認
ros2 param list /slam_toolbox
```

## 技術仕様

### ロボット仕様
- **モデル**: Hunter SE
- **駆動方式**: 4輪独立ステアリング
- **センサー**: MID-360 LiDAR（360度、30m範囲、10Hz）
- **最大速度**: 0.26 m/s
- **最大角速度**: 1.0 rad/s

### ソフトウェア構成
- **OS**: Ubuntu 22.04
- **ROS**: ROS2 Humble
- **SLAM**: slam_toolbox
- **ナビゲーション**: Nav2
- **シミュレータ**: Gazebo Classic

## サポート・問い合わせ

技術的な問題や改良要望については、プロジェクトの課題管理システムを利用してください。

---

**最終更新**: 2025年9月
**バージョン**: 1.0
**作成者**: Claude Code AI Assistant