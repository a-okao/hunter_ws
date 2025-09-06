# slam_toolbox パラメータ調整ガイド

## ファイル場所
```
/home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/config/nav2/nav2_params.yaml
```
**設定箇所**: `slam_toolbox:` セクション（行352～417）

## 主要パラメータ調整項目

### 1. マップ品質関連パラメータ

| パラメータ名 | 現在値 | 効果 | 調整方法 |
|-------------|--------|------|---------|
| `resolution` | 0.05 | マップ解像度(m/pixel) | 小さくすると高精度、大きくすると軽量化 |
| `max_laser_range` | 20.0 | LiDAR最大有効範囲(m) | センサー仕様に合わせて調整 |
| `minimum_travel_distance` | 0.5 | マップ更新最小移動距離(m) | 小さくすると頻繁更新、大きくすると軽量化 |
| `minimum_travel_heading` | 0.5 | マップ更新最小回転角度(rad) | 小さくすると頻繁更新、大きくすると軽量化 |

### 2. スキャンマッチング精度パラメータ

| パラメータ名 | 現在値 | 効果 | 調整方法 |
|-------------|--------|------|---------|
| `correlation_search_space_dimension` | 0.5 | 相関検索範囲(m) | 大きくすると精度向上、小さくすると高速化 |
| `correlation_search_space_resolution` | 0.01 | 相関検索精度(m) | 小さくすると高精度、大きくすると高速化 |
| `loop_match_minimum_response_fine` | 0.45 | ループクロージャ感度 | 小さくすると敏感、大きくすると保守的 |
| `link_match_minimum_response_fine` | 0.1 | スキャンマッチング感度 | 小さくすると敏感、大きくすると保守的 |

### 3. 計算負荷調整パラメータ

| パラメータ名 | 現在値 | 効果 | 調整方法 |
|-------------|--------|------|---------|
| `throttle_scans` | 1 | スキャン間引き率 | 大きくすると軽量化（1=間引きなし） |
| `map_update_interval` | 5.0 | マップ更新間隔(秒) | 大きくすると軽量化 |
| `transform_publish_period` | 0.02 | TF配信間隔(秒) | 大きくすると軽量化 |
| `scan_buffer_size` | 10 | スキャンバッファサイズ | 大きくすると精度向上、小さくすると軽量化 |

### 4. ループクロージャパラメータ

| パラメータ名 | 現在値 | 効果 | 調整方法 |
|-------------|--------|------|---------|
| `do_loop_closing` | true | ループクロージャ有効/無効 | falseで無効化 |
| `loop_search_maximum_distance` | 3.0 | ループ検索最大距離(m) | 大きくするとループ検出範囲拡大 |
| `loop_match_minimum_chain_size` | 10 | ループ検出最小チェーンサイズ | 大きくすると保守的 |

### 5. 環境適応パラメータ

| パラメータ名 | 現在値 | 効果 | 調整方法 |
|-------------|--------|------|---------|
| `minimum_time_interval` | 0.5 | 処理最小時間間隔(秒) | 大きくすると軽量化 |
| `transform_timeout` | 0.2 | TFタイムアウト(秒) | 大きくすると寛容、小さくすると厳格 |
| `use_scan_matching` | true | スキャンマッチング使用 | falseで無効化（非推奨） |
| `use_scan_barycenter` | true | スキャン重心使用 | falseで無効化 |

## 環境別推奨調整

### 高精度マッピング重視
```yaml
resolution: 0.02
minimum_travel_distance: 0.2
correlation_search_space_resolution: 0.005
map_update_interval: 2.0
```

### 計算負荷軽減重視
```yaml
resolution: 0.1
throttle_scans: 2
map_update_interval: 10.0
transform_publish_period: 0.05
scan_buffer_size: 5
```

### 動的環境対応
```yaml
minimum_travel_distance: 0.3
minimum_time_interval: 0.3
correlation_search_space_dimension: 0.8
loop_search_maximum_distance: 5.0
```

## パラメータ変更手順

1. **ファイル編集**
   ```bash
   nano /home/iskhas/hunter_ws/src/ugv_sim/hunter_se/hunter_se_gazebo/config/nav2/nav2_params.yaml
   ```

2. **パッケージ再ビルド**
   ```bash
   cd /home/iskhas/hunter_ws
   colcon build --packages-select hunter_se_gazebo
   source install/setup.bash
   ```

3. **動作確認**
   ```bash
   ros2 launch hunter_se_gazebo hunter_se_simulation_nav2.launch.py
   ```

## 調整のコツ

### マッピング品質が悪い場合
- `resolution` を小さく（0.02-0.03）
- `correlation_search_space_resolution` を小さく（0.005）
- `minimum_travel_distance` を小さく（0.2-0.3）

### 処理が重い場合
- `throttle_scans` を大きく（2-3）
- `map_update_interval` を大きく（8.0-10.0）
- `scan_buffer_size` を小さく（5-8）

### ループクロージャが効かない場合
- `loop_match_minimum_response_fine` を小さく（0.3-0.4）
- `loop_search_maximum_distance` を大きく（5.0-8.0）
- `loop_match_minimum_chain_size` を小さく（6-8）

### 動的環境で問題がある場合
- `minimum_time_interval` を小さく（0.2-0.3）
- `transform_timeout` を大きく（0.5-1.0）
- `correlation_search_space_dimension` を大きく（0.8-1.0）

## よく使用するデバッグコマンド

```bash
# SLAMの状態確認
ros2 topic list | grep slam

# マップトピック確認
ros2 topic echo /map --once

# TF確認
ros2 run tf2_tools view_frames

# ノード確認
ros2 node list | grep slam

# パラメータ確認
ros2 param list /slam_toolbox

# 特定パラメータの値確認
ros2 param get /slam_toolbox resolution
```

## 注意事項

1. **パラメータ変更後は必ずビルドが必要**
2. **resolution変更は既存マップとの互換性に影響**
3. **計算負荷パラメータは環境性能に応じて調整**
4. **ループクロージャは環境の特徴によって効果が変わる**