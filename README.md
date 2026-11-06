# TIRT_new_update

ROS 2 Humble 麥克納姆輪送餐比賽工作區，同步至 2026-11-06 目前實車工作區的程式、地圖與設定。包含終端機比賽流程、Nav2、RViz、STM32 底盤、EKF、光達、車身遮蔽、相機／QR、點位記錄，以及 tf2 鎖死修正庫。

## 環境與依賴

目標環境為 Ubuntu 22.04 arm64／aarch64、ROS 2 Humble。內附的 `tf2_0.25.24/lib/libtf2.so` 是官方 arm64 套件的修正核心庫；不可直接拿到 x86 電腦使用。它只在本專案導航啟動的子程序內載入，不覆蓋系統套件。版權與來源見 `tf2_0.25.24/COPYRIGHT`、`PROVENANCE.json`。

需已安裝 ROS 2 Humble 與 colcon。新機器可先安裝主要依賴（會修改系統，請由操作人執行）：

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-yaml \
  python3-serial python3-numpy python3-opencv python3-pil python3-pil.imagetk python3-tk \
  libpcap-dev libboost-thread-dev libzbar-dev libopencv-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-rviz2 ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher ros-humble-xacro ros-humble-laser-filters \
  ros-humble-usb-cam ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-pcl-conversions ros-humble-diagnostic-updater ros-humble-rosbag2
```

若有其他缺少的依賴，可在本目錄執行 `rosdep install --from-paths src --ignore-src -r -y --rosdistro humble`；先完成 rosdep 初始化。原專案部分套件的 license 欄位尚為 TODO，保留原始聲明，未另行宣稱授權。

## 編譯

```bash
git clone https://github.com/keys-lll/TIRT_new_update.git "$HOME/TIRT_new_update"
cd "$HOME/TIRT_new_update"
bash scripts/build.sh
```

請使用上述英文工作區路徑；本次測試中 Humble 的 rosidl 在中文父目錄下產生介面時會出現路徑解析錯誤。

## 建圖與儲存地圖

在已完成編譯的 ROS 2 Humble 實車上操作。先停止導航、送餐任務與其他底盤／光達啟動程序；建圖 launch 本身會啟動底盤、光達、EKF、車身雷射遮蔽、SLAM Toolbox 與 RViz。建圖期間由鍵盤遙控移動，不啟動 `bash scripts/run.sh nav` 或送餐任務。

終端機 1：啟動建圖。

```bash
cd "$HOME/TIRT_new_update"
source scripts/env.sh
ros2 launch mecanum_navigation mapping.launch.py
```

預設使用原實車的 USB serial-by-id。換機時可在 launch 命令後加入 `stm32_port:=/dev/你的底盤裝置 lidar_port:=/dev/你的光達裝置`。若硬體已由本工作區另外啟動，才加上 `start_hardware:=false`，避免重複開啟裝置。

RViz 的 Fixed Frame 設為 `map`，確認 Map 顯示 `/map`、LaserScan 顯示 `/scan`。地圖解析度為 0.05 m/pixel（每格 5 公分），更新間隔為 5 秒；剛啟動時可稍等地圖出現。設定檔為 `src/mecanum_navigation/config/slam_toolbox.yaml`。

終端機 2：啟動鍵盤遙控，操作時讓此終端機保持焦點。

```bash
cd "$HOME/TIRT_new_update"
source scripts/env.sh
ros2 run keyboard_control keyboard_control
```

| 按鍵 | 動作 |
|---|---|
| `i`／`,` | 前進／後退 |
| `a`／`d` | 向左／向右平移 |
| `j`／`l` | 預設 CommonMode 下向左／向右旋轉 |
| 空白鍵／`k` | 停止所有方向 |
| `z`／`q` | 降低／提高速度上限 10% |
| `b` | 切換 CommonMode／OmniMode，部分按鍵含義會改變 |
| Ctrl+C | 停止並結束遙控 |

預設平移速度為 0.1 m/s、旋轉速度為 0.3 rad/s。慢速走過各通道與桌邊，讓光達看清牆角，再繞回已走過的區域完成閉環。確認牆線沒有明顯重影或扭曲後，先停車並等待地圖更新。

終端機 3：保持建圖程序執行中，先用新檔名儲存地圖。

```bash
cd "$HOME/TIRT_new_update"
source scripts/env.sh
ros2 run nav2_map_server map_saver_cli \
  -f "$TIRT_WORKSPACE/maps/mecanum_map_new" \
  --ros-args -p save_map_timeout:=10.0 \
  -p map_subscribe_transient_local:=true
```

確認命令成功，且 `maps/mecanum_map_new.yaml` 與 `maps/mecanum_map_new.pgm` 都已產生，再在遙控與建圖終端機按 Ctrl+C 結束。YAML 內的 `image`、`resolution`、`origin` 必須與圖檔一起保留；僅關閉建圖視窗不會自動儲存地圖。若儲存逾時，先確認建圖仍在執行、`/map` 有資料，再重試。

要先測試新地圖，可在停止建圖後另開終端機執行：

```bash
cd "$HOME/TIRT_new_update"
source scripts/env.sh
ros2 launch mecanum_navigation navigation.launch.py \
  map:="$TIRT_WORKSPACE/maps/mecanum_map_new.yaml" \
  use_keepout:=false use_rviz:=true
```

這裡暫時關閉舊禁行區，並由 RViz 的 **2D Pose Estimate** 設定實際車位與朝向。重新建圖後座標原點、方向或尺寸可能改變，請重新記錄 start、prep、各桌點位，並依新地圖尺寸、解析度與原點重畫／對齊 keepout；不要直接沿用舊座標與遮罩。

正式改用新地圖時，更新 `scripts/run.sh` 內的 `map:=` 路徑，並確認 `maps/keepout.yaml` 與新遮罩對應；或在備份舊地圖後，將新地圖部署為原本的 `maps/mecanum_map.yaml` 與正確的 `image` 圖檔。完成點位與遮罩更新後，再使用下方的導航／任務啟動流程。

## 分開啟動

先停止其他工作區的導航與任務，將車放在已記錄的 start 位置與朝向。兩個終端機均進入本儲存庫目錄。

終端機 1：導航、RViz、rosbag 與日誌

```bash
bash scripts/run.sh nav
```

終端機 2：比賽終端機

```bash
bash scripts/run.sh mission
```

任務載入所有記錄點，輸入 1～5 號桌後按 Enter 出發，不先導航回 start。導航啟動腳本仍以 start 初始化定位，因此啟動導航時車子應放在 start。第一趟桌邊掃到 `none` 照常前往 prep；第二趟掃到 `none` 返回 start 後直接重新選桌，程式保留。每次 prep 都要按 Enter 才出發，沒有 6 秒自動出發。啟動 Python 任務程式時相機即開始運作，持續到程式結束；只有點餐／加點掃碼提示期間才接受 QR，掃到 QR 或按 Enter 放行。加點步驟直接 Enter 表示不加點並回 start。送完加點後的最後桌邊步驟按 Enter 返回 start，不啟動 QR。`q` 加 Enter 中止任務，Ctrl+C 停止程式。相機不需第三個終端機。

日誌與錄製資料存於 `test_logs/日期時間_nav_*/`；`latest_test.txt` 指向最近一次導航。每次錄製保留當次導航參數及點位快照。請在導航終端機 Ctrl+C 結束，以完成 bag 寫入。

## 手動調點與記錄

```bash
nano src/car_waypoints/config/waypoints.yaml
```

第一行必須為 `waypoints:`。x、y 為地圖公尺座標，yaw 為弧度。新任務重新載入點位，原本已送出的導航目標不會隨檔案改動。

需要重新記錄目前車位時，先啟動導航並確認定位：

```bash
source scripts/env.sh
ros2 run car_waypoints wp_record
```

底盤遙控（不要與進行中的自主導航同時使用）：

```bash
source scripts/env.sh
ros2 run keyboard_control keyboard_control
```

## 目前主要設定

本次上傳同步目前實車工作區；局部避障權重最終選用 2.4。

| 項目 | 設定 |
|---|---|
| 最大平移／旋轉速度 | 0.25 m/s／1.4 rad/s |
| 控制頻率／局部地圖更新頻率 | 20 Hz／10 Hz |
| 全域膨脹半徑／衰減係數 | 0.20 m／0.5 |
| 局部膨脹半徑／衰減係數 | 0.35 m／1.0 |
| 局部地圖網格 | 0.025 m |
| 全域／局部 footprint_padding | 0／0.015 m |
| 車身長／寬 | 0.25／0.32 m |
| ObstacleFootprint／PathAlign／PathDist 權重 | 2.4／8／8 |
| GoalAlign／GoalDist／RotateToGoal 權重 | 24／24／32 |
| X／Y／角速度取樣數 | 15／15／15 |
| 預測時間 | 1.7 秒 |
| 平移／旋轉減速度絕對值 | 0.3 m/s²／1.5 rad/s² |
| 一般／prep 位置容許值 | 0.05／0.05 m |
| 一般／prep 朝向容許值 | 3.14 rad |
| A／B 位置容許值 | 0.03 m；不要求對正朝向 |
| 無進展容許時間／控制失敗容許時間 | 15／0.3 秒 |

Nav2 設定在 `src/mecanum_navigation/config/nav2_params.yaml`。啟動檔會將一般 DWB 設定合併至 prep 與 A/B 控制器；A/B 只使用 Oscillation、ObstacleFootprint、PathDist、GoalDist 評分。此版本保留 Oscillation，額外 collision monitor 未啟動。這些參數不包含移動目標速度追蹤或預測。

任務進備餐區必經 `b → a → prep`，離開必經 `prep → a → b → 目的地`。A/B 點位必須存在，不可跳過失敗的中繼點。點位檔是 `src/car_waypoints/config/waypoints.yaml`。

目前 GUI 同步自實車版本，可在已啟動導航的另一終端機執行：

```bash
source scripts/env.sh
ros2 run auto mission_gui
```

GUI 提供數字鍵操作與持續相機預覽；實際可用操作以當前畫面提示為準。終端任務仍可用 `bash scripts/run.sh mission`。

## 地圖與禁行區

`maps/mecanum_map.yaml` 與 PGM 同步自目前使用的 `/home/leo/TIRT_ws/map`，圖像為 312 × 398 像素，解析度 0.05 m，原點 `[-7.18, -11.5, 0]`。圖檔、原點與點位需配對使用。`maps/mecanum_map_org.pgm` 保留為既有歷史資料，不是導航腳本載入的圖檔。

禁行區預設關閉（`use_keepout:=false`），與目前實車啟動行為一致。保留的 `maps/keepout.yaml`／PGM 屬於舊地圖座標，不能直接視為已對齊目前主地圖。先重新製作或核對遮罩後，才以 `use_keepout:=true keepout_mask:=/absolute/path/to/aligned_mask.yaml` 啟用。此版本沒有舊版的 keepout 額外膨脹層。`src/mecanum_navigation/maps/1031i_keepout.*` 亦為保留資料，預設不載入。

透過腳本啟動會設定 `TIRT_WORKSPACE`，點位、地圖與 tf2 修正庫使用本儲存庫檔案；直接執行 ROS 指令前請先 `source scripts/env.sh`。硬體 USB serial-by-id 與相機裝置沿用原機設定。

## 本次驗證範圍

上傳前核對來源快照、Python 語法、YAML/XML 解析、shell 語法、地圖影像與點位，以及 prep/A/B 控制器的參數繼承。未啟動硬體、未進行實車避障測試，也未重新完整編譯。舊版 README 的合成障礙通過率與間距數據不適用於本次地圖及設定。歷史 TF 修正報告見 `docs/tf_deadlock_debug_20261102.md`。
