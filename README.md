# Hunter SE SLAM & 自律走行システム

Hunter SEロボットを使用したSLAM（Simultaneous Localization and Mapping）と自律走行システムのマニュアルです。

## システム概要

- **ロボット**: Hunter SE（4輪独立ステアリング）
- **センサー**: MID-360 LiDAR
- **環境**: ROS2 Humble + Gazebo + Nav2
- **機能**: SLAM、自律ナビゲーション、マップ作成・保存

## 目次

1. [環境設定](#環境設定)
2. [パッケージ構成](#パッケージ構成)
3. [SLAMによるマップ作成](#slamによるマップ作成)
4. [自律ナビゲーション](#自律ナビゲーション)
5. [トラブルシューティング](#トラブルシューティング)
6. [パラメータ調整](#パラメータ調整)

## 環境設定

### 必要な依存関係

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

### ワークスペースのビルド

```bash
cd /home/iskhas/hunter_ws
colcon build --packages-select hunter_se_description hunter_se_gazebo
source install/setup.bash
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

## SLAMによるマップ作成

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

## 自律ナビゲーション

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

## 利用可能な環境

### 1. 工場環境（factory.world）
- 30m×30mの工場フロア
- 製造機械、支柱、静的障害物
- 複雑な通路とコーナー

### 2. 住宅環境（house.world）
- 基本的な住宅レイアウト
- シンプルな構造

## よく使用するコマンド集

```bash
# システム起動
ros2 launch hunter_se_gazebo hunter_se_simulation_nav2.launch.py

# ナビゲーションテスト
ros2 launch hunter_se_gazebo hunter_se_navigation_test.launch.py

# 手動操作
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# マップ保存
ros2 run nav2_map_server map_saver_cli -f ~/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/maps/my_map

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