# タートルシム画像描画システム ガイド

## 概要
- 画像を線分に変換して `turtlesim` で描画する一連の ROS 2 パッケージの使い方をまとめました。
- 主要パッケージ
  - `image_to_turtle`: 画像をダウンロード/読込し、線検出してタートルで描画する統合ノード群。
  - `turtle_artist`: 画像輪郭を抽出してタートルでトレースするシンプル実装。
  - `image_drawing_pkg`: 画像描画のひな型（最小実装）。

## 前提
- ROS 2 Jazzy 環境 `/opt/ros/jazzy` が使えること。
- 依存: `opencv-python`, `numpy`, `requests`（`image_to_turtle/setup.py` が pip 依存を宣言）。
- 作業ディレクトリ: `/home/matsuda/ros2_dds_dev_ws/matsuda/ros2_comms_ws`

## 初回ビルド
```bash
cd /home/matsuda/ros2_dds_dev_ws/matsuda/ros2_comms_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select image_to_turtle turtle_artist
source install/setup.bash
```

## クイック実行（推奨: 2ターミナル）
### ターミナルA: turtlesim 起動
```bash
cd /home/matsuda/ros2_dds_dev_ws/matsuda/ros2_comms_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run turtlesim turtlesim_node
```
このターミナルは起動したままにします。

### ターミナルB: 画像→線→描画（simple_demo）
```bash
cd /home/matsuda/ros2_dds_dev_ws/matsuda/ros2_comms_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run image_to_turtle simple_demo
```
- サンプル画像を取得し、線検出して `turtlesim` に描画します。
- 画像表示コマンド（display/eog等）が環境に無い場合、表示はスキップされますが描画処理は進みます。

## その他の実行例
- インタラクティブに URL/ローカル画像を指定:
```bash
ros2 run image_to_turtle image_to_turtle_main
```
- 輪郭トレースのシンプル版:
```bash
ros2 run turtle_artist draw_shape
```

## トラブルシューティング
- `spawnサービスを待機中...` が続く / `service_is_ready` エラー:
  - `turtlesim_node` が起動していない可能性。ターミナルAを起動してから再実行。
- 画像が表示されない:
  - GUI環境が無い or ビューアが未インストール。描画自体は継続するので無視可。
- 再ビルドしたのに反映されない:
  - `source install/setup.bash` を再度実行。

## 停止
- 各ターミナルで `Ctrl+C`。
- 背景で残った場合は `pkill -f turtlesim` や `pkill -f image_to_turtle` で停止可能。


