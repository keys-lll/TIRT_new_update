# car_waypoints

開車到位置、輸入名稱就存成點位；之後用 `go <名稱>` 一次開到一個點。
點位存在 `src/car_waypoints/config/waypoints.yaml`（地圖座標，單位 m / rad），可以直接 git commit。

## 建置

```bash
source /opt/ros/humble/setup.bash
cd ~/mecanum_ros2
colcon build --packages-select car_waypoints
source install/setup.bash
```

## 1. 記錄點位（需要 map → base_link 的 TF，也就是 Nav2 / AMCL 已經定位）

```bash
ros2 run car_waypoints wp_record
```

用鍵盤把車開進框內、擺好方向，輸入名稱：

| 指令 | 作用 |
|---|---|
| `start` | 把車子目前位置存成 `start`（已存在就覆寫） |
| `del t1` | 刪除 |
| `list` | 列出全部 |
| `q` | 離開 |

名稱：英文字母開頭，只用英數字和底線。依名稱在 RViz 畫出簡章尺寸的框：
`start`、`prep` 90 cm，`t1`…`t5` 75 cm，其他 33 cm。
RViz 加一個 MarkerArray，topic 選 `/waypoint_markers`。

建議命名：`start`（出發區）、`prep`（備餐區）、`t1`…`t5`（五個紅框）。

進出備餐區的中繼點：`a` 是備餐區前方、`b` 是 5 號桌旁。建圖時 SLAM 已提供 map → base_link TF，也能執行 wp_record 記錄。把車開到每個位置，分別輸入 `a`、`b`；大寫 `A`、`B` 也會統一存成小寫。輸入 `list` 檢查、再次輸入同名點可覆寫。A、B 不必刻意對準車頭；檔案仍保留當下 yaw 以維持既有格式。這裡只新增記錄入口，任務必經路線與忽略到點朝向需由任務控制器另外設定。

## 2. 前往點位

```bash
ros2 launch car_navigation2 navigation2.launch.py   # 先把 Nav2 開起來
ros2 run car_waypoints wp_go
```

| 指令 | 作用 |
|---|---|
| `init start` | 告訴 AMCL 車子現在就在 `start`（車子要真的放在那裡） |
| `go t3` | 開到 `t3`，到了就停下等下一個指令，並印出花了幾秒 |
| `list` | 列出全部（每次都重讀檔案，`wp_record` 新存的點馬上能用） |
| Ctrl+C | 取消目前的 `go`，回到提示符 |
| `q` | 離開 |

簡章單趟上限：無障礙 90 秒、有障礙 120 秒，超過會在到達訊息後面標出警告。

## 參數

兩個程式都可以用 `--ros-args -p file:=<路徑> -p map_frame:=map -p base_frame:=base_link` 覆寫。

## 測試

```bash
cd ~/mecanum_ros2/src/car_waypoints && python3 -m pytest
```

整合測試用假的 TF 和假的 NavigateToPose server，只在本機（`ROS_LOCALHOST_ONLY=1`、domain 87），
不會影響實車上跑的節點。
