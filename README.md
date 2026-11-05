# Ros2_tirt

ROS 2 Humble 麥克納姆輪送餐比賽工作區，同步至 2026-11-05 目前實車工作區的程式、地圖與設定。包含終端機比賽流程、Nav2、RViz、STM32 底盤、EKF、光達、車身遮蔽、相機／QR、點位記錄，以及 tf2 鎖死修正庫。

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
git clone https://github.com/keys-lll/Ros2_tirt.git "$HOME/Ros2_tirt"
cd "$HOME/Ros2_tirt"
bash scripts/build.sh
```

請使用上述英文工作區路徑；本次測試中 Humble 的 rosidl 在中文父目錄下產生介面時會出現路徑解析錯誤。

## 建圖與儲存地圖

在已完成編譯的 ROS 2 Humble 實車上操作。先停止導航、送餐任務與其他底盤／光達啟動程序；建圖 launch 本身會啟動底盤、光達、EKF、車身雷射遮蔽、SLAM Toolbox 與 RViz。建圖期間由鍵盤遙控移動，不啟動 `bash scripts/run.sh nav` 或送餐任務。

終端機 1：啟動建圖。

```bash
cd "$HOME/Ros2_tirt"
source scripts/env.sh
ros2 launch mecanum_navigation mapping.launch.py
```

預設使用原實車的 USB serial-by-id。換機時可在 launch 命令後加入 `stm32_port:=/dev/你的底盤裝置 lidar_port:=/dev/你的光達裝置`。若硬體已由本工作區另外啟動，才加上 `start_hardware:=false`，避免重複開啟裝置。

RViz 的 Fixed Frame 設為 `map`，確認 Map 顯示 `/map`、LaserScan 顯示 `/scan`。地圖解析度為 0.05 m/pixel（每格 5 公分），更新間隔為 5 秒；剛啟動時可稍等地圖出現。設定檔為 `src/mecanum_navigation/config/slam_toolbox.yaml`。

終端機 2：啟動鍵盤遙控，操作時讓此終端機保持焦點。

```bash
cd "$HOME/Ros2_tirt"
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
cd "$HOME/Ros2_tirt"
source scripts/env.sh
ros2 run nav2_map_server map_saver_cli \
  -f "$TIRT_WORKSPACE/maps/mecanum_map_new" \
  --ros-args -p save_map_timeout:=10.0 \
  -p map_subscribe_transient_local:=true
```

確認命令成功，且 `maps/mecanum_map_new.yaml` 與 `maps/mecanum_map_new.pgm` 都已產生，再在遙控與建圖終端機按 Ctrl+C 結束。YAML 內的 `image`、`resolution`、`origin` 必須與圖檔一起保留；僅關閉建圖視窗不會自動儲存地圖。若儲存逾時，先確認建圖仍在執行、`/map` 有資料，再重試。

要先測試新地圖，可在停止建圖後另開終端機執行：

```bash
cd "$HOME/Ros2_tirt"
source scripts/env.sh
ros2 launch mecanum_navigation navigation.launch.py \
  map:="$TIRT_WORKSPACE/maps/mecanum_map_new.yaml" \
  keepout_mask:='' use_rviz:=true
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

| 項目 | 設定 |
|---|---|
| 平移速度上限 | 0.35 m/s |
| 旋轉速度上限 | 0.6 rad/s |
| 控制頻率 | 20 Hz |
| 一般點位位置／方向容許誤差 | 0.06 m／5 度 |
| prep goal checker 位置／方向容許誤差 | 0.05 m／5 度 |
| prep DWB 位置容許值 | 0.05 m（與 checker 一致） |
| 全域膨脹半徑／衰減係數 | 0.70 m／2.0 |
| 局部膨脹半徑／衰減係數 | 0.50 m／15.0 |
| 局部 costmap 網格 | 0.025 m；主地圖仍為 0.05 m/pixel |
| 局部 costmap 更新頻率 | 10 Hz |
| DWB ObstacleFootprint／PathAlign／RotateToGoal 權重 | 3.0／8／32 |
| DWB GoalAlign／PathDist／GoalDist 權重 | 0／4／128 |
| DWB x／y／角速度取樣數 | 15／15／15 |
| DWB 預測時間 | 1.2 秒 |
| 平移／旋轉減速度絕對值 | 0.6 m/s²／3.0 rad/s²（controller 與 velocity smoother 一致） |
| RotateToGoal 角度評分前瞻 | 0.5 秒 |
| 車身 footprint | 長 0.25 m、寬 0.32 m |
| 全域／局部 footprint_padding | 0.04 m／0.015 m |
| 光達相對 base_link | x=0.12 m、z=0.23 m |

Nav2 設定為 `src/mecanum_navigation/config/nav2_params.yaml`；車身遮蔽為同目錄 `self_filter.yaml`。額外 collision monitor 未啟動，Nav2 仍使用 costmap／footprint 判斷碰撞。4 號桌直接前往記錄的 t4 停靠點。

平移上限保留 0.35 m/s，DWB 在窄路、避障及接近目標時可選擇較低速度。`GoalAlign.scale=0` 不會取消最終朝向檢查；最後仍由 RotateToGoal 與 goal checker 收斂朝向。停用 Oscillation 評分，保留 ObstacleFootprint 碰撞檢查；`failure_tolerance=1.0`、無進展容許時間 30 秒，既有暫停與復原流程仍適用。prep 現在也要求朝向接近記錄的 0 度，避免以任意朝向開始下一段窄路。

地圖為 `maps/mecanum_map.yaml` 與對應 PGM；地圖原點與圖檔必須成對保留。RViz 預設顯示 Map、TF、AMCL、雷射、全域路徑，以及全域／局部 costmap。實車 USB serial-by-id、相機 `/dev/video0` 設定沿用原機，換機時請確認 `hardware.launch.py`、`navigation.launch.py` 與相機裝置。

透過腳本啟動時會設定 `TIRT_WORKSPACE`，因此點位、修正庫與地圖使用這個儲存庫自身檔案。直接使用 ros2 命令前請先 `source scripts/env.sh`。

## 禁行區

`maps/keepout.yaml` 載入手繪的 `maps/keepout.pgm`，沿用主地圖的 0.05 m/pixel 與原點。黑色為禁行區、白色不增加禁行限制。以目前 `negate=0`、`free_thresh=0.25` 設定，灰色 205 的佔用機率約 0.196，會解讀為 free，並非 unknown；真正的 unknown 遮罩格會被 KeepoutFilter 忽略。遮罩不會清除主地圖既有的障礙物。圖中保留的黑色實體牆壁也會成為禁行區。編輯時保持圖片尺寸與位置，儲存後重新啟動導航。

`bash scripts/run.sh nav` 會同時啟動遮罩 map server、filter info server 與其 lifecycle manager，並在全域與局部 costmap 啟用 KeepoutFilter，不需另開終端機。首次修改 launch／設定後執行 `bash scripts/build.sh`。RViz 可新增 Map 顯示 `/keepout_mask`（Durability: Transient Local），再查看全域與局部 costmap 確認禁行區已加入。

全域與局部 costmap 的 filters 依序執行 `keepout_filter`、`keepout_inflation_layer`，後者使用 `nav2_costmap_2d::InflationLayer`。keepout 膨脹半徑分別為全域 0.30 m、局部 0.12 m，衰減係數皆為 3.0；此層處理加入禁行區後的 costmap。一般障礙物膨脹分別為全域 0.70 m／係數 2.0、局部 0.50 m／係數 15.0。局部 2.5 公分網格會將 0.12 m 半徑向上取整為 5 格（0.125 m）。膨脹增加周邊代價，不等同將整個緩衝區變成黑色禁行區，也不直接代表車身邊緣的離障距離。禁行區限制 Nav2 規劃與控制，不會阻擋鍵盤直接發布的底盤命令。直接啟動 navigation.launch.py 時可用 `keepout_mask:=/absolute/path/to/mask.yaml` 指定其他遮罩，或以 `keepout_mask:=''` 一起關閉禁行區及其膨脹層。

## 驗證範圍

初次整理時曾完成 10 套件編譯。本次使用最新 keepout（SHA256 前綴 `50a7e036dee1`）及 ROS 2 Humble／Nav2 1.1.20 的原生 DWB、NavFn、InflationLayer，在專案外的暫存環境記錄軌跡。20 段獨立路線與承接實際停車姿態的 20 段連續路線皆完成，未偵測到車身與佔用格重疊；一般點位位置誤差小於 6 cm，所有點位方向誤差小於 5 度。九種合成障礙情境中，八種完成到點、一種在通道完全封閉時保持暫停；最小車身邊緣間距為 17.73 cm。通道障礙移除的案例先停 12 秒，再繼續到點。

上述為理想運動、合成 costmap 的離線結果，並非實車里程計軌跡；未包含真實雷射遮蔽、定位誤差、馬達延遲或完整 BT 復原流程，不能視為任意動態障礙下固定 15 cm 距離的保證。本次未重新完整編譯專案或執行實車任務。既有單檔任務測試在修改前後皆為 62 通過、10 失敗，失敗項目相同，未擴大本次參數調整範圍去改動其原有流程。TF 修正的既有離線 A/B 證據見 `docs/tf_deadlock_debug_20261102.md`；歷史報告中的原機路徑僅供追溯。
