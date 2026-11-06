#!/usr/bin/env python3
"""
Delivery competition mission with terminal table selection.

先啟動導航，以 initial_waypoint:=start 初始化，再執行：
    ros2 run auto run_mission

終端機啟動時自動讀取 car_waypoints 的 waypoints.yaml，列出所有已記錄點位。
直接輸入已記錄的桌號（1～5），按 Enter 出發帶位，不先導航到 start。
後續依序帶位、固定餐、加點餐，第二回合換區。
點餐、加點到桌邊後開相機，掃 QR 或 Enter 放行；最後返回 start 前只按 Enter。
進備餐區：b → a → prep；離開：prep → a → b → 目的地；a/b 不對正角度。
prep 按 Enter 出發；第一趟 none 仍去 prep，第二趟 none 返回 start 後重新選桌。
q + Enter 可中止導航或等待；結束後 Enter 重新開始。
"""
import math
import os
from ament_index_python.packages import get_package_share_directory
import time
from typing import NamedTuple

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.duration import Duration
from rclpy.time import Time
from car_waypoints.store import DEFAULT_FILE, load_waypoints

# =========================== 設定（要改的都在這一區） ===========================
# 每個點是 (x, y, yaw)：地圖座標，x、y 單位公尺，yaw 單位「度」（逆時針為正）。
# 還沒量的點寫 None，程式會拒絕啟動並告訴你缺哪一個。
START = None     # 出發區
PREP = None     # 備餐區
TABLES = {                        # 1~5 號桌的紅色停等區
    1: None,
    2: None,
    3: None,
    4: None,
    5: None,
}
STOP_SECONDS = 0.0                # 到點後立即進入桌邊 QR 等待，不額外停等
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'
LEG_LIMIT = 90                    # 簡章單趟上限（秒），沒有障礙物
LEG_LIMIT_WITH_OBSTACLES = 120    # 有障礙物
# ==============================================================================

FRONT_TABLES = (1, 4)
BACK_TABLES = (2, 3, 5)
QUIT = ('q', 'quit')
MISSION_FINISHED = 2  # Current flow ended at start; terminal immediately returns to table selection.
NAMES = ('start', 'prep', 't1', 't2', 't3', 't4', 't5', 'a', 'b')


class NavResult(NamedTuple):
    """outcome is SUCCEEDED, FAILED, CANCELED or NOT_READY (never started)."""

    outcome: str
    text: str
    seconds: float = 0.0

    @property
    def ok(self):
        return self.outcome == 'SUCCEEDED'


class MissionFinished(Exception):
    """A table-side none card ended the current flow after returning to start."""


class MissionAborted(Exception):
    """The operator quit, or input ended."""


def all_points(recorded_only=False):
    """Return the recorded mission points, including prep transit points a/b."""
    points = {'start': START, 'prep': PREP, 'a': None, 'b': None}
    points.update({f't{n}': point for n, point in sorted(TABLES.items())})
    # The shared waypoint YAML stores yaw in radians; this legacy API uses degrees.
    recorded = load_waypoints(DEFAULT_FILE)
    if recorded_only:
        return {name: (p.x, p.y, math.degrees(p.yaw)) for name, p in recorded.items()}
    for name, waypoint in recorded.items():
        if name in points:
            points[name] = (waypoint.x, waypoint.y, math.degrees(waypoint.yaw))
    return points


def unset_points(points):
    """Return the names of points still set to None."""
    return [name for name, point in points.items() if point is None]


def pose_from_point(point, frame_id, stamp):
    """Build the Nav2 goal for an (x, y, yaw in degrees) point."""
    x, y, yaw_degrees = point
    yaw = math.radians(yaw_degrees)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def arrival_report(name, outcome, seconds):
    """Describe a finished goal; outcome is a TaskResult name."""
    if outcome == 'CANCELED':
        return f'已取消前往 {name}（{seconds:.1f} 秒）'
    if outcome != 'SUCCEEDED':
        return f'前往 {name} 失敗（{outcome}，{seconds:.1f} 秒）'
    text = f'到達 {name}，花了 {seconds:.1f} 秒'
    if seconds > LEG_LIMIT_WITH_OBSTACLES:
        return f'{text}  ⚠ 超過有障礙單趟上限 {LEG_LIMIT_WITH_OBSTACLES} 秒'
    if seconds > LEG_LIMIT:
        return f'{text}  ⚠ 超過無障礙單趟上限 {LEG_LIMIT} 秒'
    return text


def allowed_second_tables(first):
    """Return the tables round two may draw: the other zone than round one."""
    return BACK_TABLES if first in FRONT_TABLES else FRONT_TABLES


class Mission:
    """
    The delivery flow: three trips per round, driven by operator input.

    Navigation, input, output and sleeping are injected, so the flow runs without
    ROS and a camera can replace the keyboard later.
    """

    def __init__(self, navigate, ask, say, sleep, stop_seconds=STOP_SECONDS, ready=None, prepare_start=None, start_before_table=False):
        self.navigate = navigate
        self.ask = ask
        self.say = say
        self.sleep = sleep
        self.stop_seconds = stop_seconds
        self.prepare_start = prepare_start
        self.start_before_table = start_before_table
        self.last_waypoint = None
        # 已經填了座標的點；None 表示所有任務點都能用
        self.ready = set(NAMES if ready is None else ready)

    def ask_line(self, prompt):
        """Ask the operator; q (or end of input) aborts the mission."""
        try:
            answer = self.ask(prompt).strip()
        except EOFError:
            raise MissionAborted('輸入結束')
        if answer.lower() in QUIT:
            raise MissionAborted('操作手結束')
        return answer

    def ask_table_side(self, prompt):
        """A second-trip none QR/card returns to start and ends the current flow."""
        answer = self.ask_line(prompt)
        if answer.casefold() == 'none':
            self.say('桌邊收到 none，直接返回 start，之後重新選桌')
            self.go('start')
            raise MissionFinished('已返回 start，請重新選桌')
        return answer

    def ask_table(self, prompt, allowed=(1, 2, 3, 4, 5), allow_blank=False):
        """Ask for a table number until it is one of `allowed`; None if blank is allowed."""
        while True:
            answer = self.ask_line(prompt)
            if not answer and allow_blank:
                return None
            if answer.isdecimal() and 1 <= int(answer) <= 5:
                table = int(answer)
                if f't{table}' not in self.ready:
                    self.say(f't{table} 還沒設定座標，請先用 wp_record 記錄此點')
                elif table not in allowed:
                    self.say(f'第二回合要換區，只能抽 {"、".join(map(str, allowed))} 桌')
                else:
                    return table
            else:
                self.say('請輸入 1~5 的桌號')

    def run(self):
        """Take custom routes or run the competition rounds, until the operator quits."""
        self.say(f'可用的點：{", ".join(n for n in NAMES if n in self.ready)}')
        unset = [n for n in NAMES if n not in self.ready]
        if unset:
            self.say(f'還沒設定座標：{", ".join(unset)}（請用 wp_record 記錄）')
        while True:
            answer = self.ask_line(
                '要去哪些點？（空白分開，例如 start t1 prep；直接按 Enter = 跑比賽流程；q 結束）> ')
            if not answer:
                if self.run_rounds():
                    return
                continue
            names = self.parse_route(answer)
            if names:
                self.run_route(names)

    def parse_route(self, text):
        """Return the point names typed, or None after saying what is wrong."""
        names = []
        for word in text.split():
            name = word.lower()
            if name not in NAMES:
                self.say(f'沒有「{word}」這個點，可用的有：{", ".join(NAMES)}')
                return None
            if name not in self.ready:
                self.say(f'{name} 還沒設定座標，請先用 wp_record 記錄此點')
                return None
            names.append(name)
        return names

    def run_route(self, names):
        """Visit the points in order, waiting for Enter before leaving for each one."""
        for i, name in enumerate(names, 1):
            self.ask_line(f'按 Enter 前往 {name}（第 {i}/{len(names)} 站，q 結束）> ')
            self.go(name)
        self.say(f'路線完成：{" → ".join(names)}')

    def run_rounds(self, select_first=False):
        """
        Run round one at any table, then optionally round two from the other zone.

        Return False, having said why, when the points needed are not set.
        """
        missing = [n for n in ('start', 'prep', 'a', 'b') if n not in self.ready]
        if missing:
            self.say(f'比賽流程需要 {"、".join(missing)} 的座標，請先用 wp_record 記錄此點')
            return False
        usable = tuple(n for n in range(1, 6) if f't{n}' in self.ready)
        if not usable:
            self.say('1~5 號桌都還沒設定座標，比賽流程沒辦法跑')
            return False
        if not select_first and self.prepare_start is not None:
            self.prepare_start()
        prompt = (f'要去哪一桌？（已記錄桌號 {"、".join(map(str, usable))}；q 結束）> '
                  if select_first else
                  f'第一回合抽到幾桌（可選 {"、".join(map(str, usable))}，q 結束）> ')
        first = self.ask_table(prompt, allowed=usable)
        self.say(f'已選 {first} 號桌，按 Enter 出發')
        self.run_round(first)
        allowed = tuple(n for n in allowed_second_tables(first) if n in usable)
        if not allowed:
            self.say('第二回合要抽另一區，但那一區的桌子都還沒設定座標，所以不跑第二回合')
            return True
        second = self.ask_table(
            f'第二回合抽到幾桌（只能 {"、".join(map(str, allowed))}；直接按 Enter 結束）> ',
            allowed=allowed, allow_blank=True)
        if second is not None:
            self.run_round(second)
        return True

    def go(self, name):
        """Route every prep entry/exit through the recorded transit points."""
        if name == 'prep' and self.last_waypoint != 'prep':
            route = {'a': ['prep'], 'b': ['a', 'prep']}.get(
                self.last_waypoint, ['b', 'a', 'prep'])
        elif self.last_waypoint == 'prep' and name != 'prep':
            route = ['a'] if name == 'a' else ['a', 'b']
            if name not in ('a', 'b'):
                route.append(name)
        else:
            route = [name]
        missing = [point for point in route if point not in self.ready]
        if missing:
            raise MissionAborted('缺少必經點 ' + '、'.join(missing) + '，請先用 wp_record 記錄')
        if len(route) > 1:
            self.say('備餐區必經路線：' + ' → '.join(route))
        for point in route:
            self._go_direct(point)

    def _go_direct(self, name):
        """Visit one point; failed mandatory transit points cannot be skipped."""
        while True:
            result = self.navigate(name)
            self.say(result.text)
            if result.ok:
                self.last_waypoint = name
                return
            while True:
                prompt = ('中繼點未到達。[r] 重試  [q] 結束 > ' if name in ('a', 'b') else
                          '這一步沒成功。[r] 重試  [s] 跳過（我已手動把車放到位）  [q] 結束 > ')
                choice = self.ask_line(prompt).lower()
                if choice == 's' and name not in ('a', 'b'):
                    self.last_waypoint = name
                    return
                if choice == 'r':
                    break
                self.say('請輸入 r 或 q' if name in ('a', 'b') else '請輸入 r、s 或 q')

    def hold(self):
        """Optionally dwell; zero proceeds directly to the table-side QR prompt."""
        if self.stop_seconds <= 0:
            return
        self.say(f'靜止 {self.stop_seconds:g} 秒…')
        self.sleep(self.stop_seconds)

    def wait_at_prep(self, message):
        """Wait for the operator to finish loading; q remains available."""
        self.say(message)
        while self.ask_line('備餐完成後按 Enter 出發（q 結束）> '):
            self.say('請直接按 Enter 出發，或輸入 q 結束')

    def run_round(self, table):
        """Trip 1 seat the guest, trip 2 deliver the set meal, trip 3 deliver the extra order."""
        spot = f't{table}'

        self.say(f'===== 第一趟 帶位到 {table} 號桌 =====')
        self.ask_line(f'按 Enter 出發帶位到 {table} 號桌（q 結束）> ')
        self.go(spot)
        self.hold()
        order = self.ask_line('請裁判給點餐卡，輸入點餐內容（可直接按 Enter 略過）> ')
        self.go('prep')

        self.say(f'===== 第二趟 送固定餐到 {table} 號桌 =====')
        self.wait_at_prep(f'請把餐點放上托盤（點餐：{order or "未輸入"}），即將出發送到 {table} 號桌')
        self.go(spot)
        self.hold()
        extra = self.ask_table_side('請裁判給加點卡，輸入加點項目（可樂 / 橘子果汁 / 白開水；Enter 略過）> ')
        if not extra.strip():
            self.say('沒有掃到加點 QR Code，返回起始點')
            self.go('start')
            self.say(f'{table} 號桌這個回合完成')
            return
        self.go('prep')

        self.say(f'===== 第三趟 送加點餐到 {table} 號桌 =====')
        self.wait_at_prep(f'請放上加點餐點（{extra or "未輸入"}），即將出發送到 {table} 號桌')
        self.go(spot)
        self.hold()
        while self.ask_line('桌邊作業完成，按 Enter 返回 start（q 結束）> '):
            self.say('請直接按 Enter 確認返回 start，或輸入 q 結束')
        self.go('start')
        self.say(f'{table} 號桌這個回合完成')


class Navigator(BasicNavigator):
    """
    Nav2 client that reports whether a goal succeeded.

    The TF listener runs on its own node and thread: BasicNavigator spins its own
    node, and two executors must not spin the same node.
    """

    def __init__(self):
        super().__init__('run_mission')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, None, spin_thread=True)

    def close(self):
        """Stop the listener thread (not a daemon, it would block exit) and the node."""
        self.listener.executor.shutdown()
        self.listener.dedicated_listener_thread.join(timeout=3.0)
        self.listener.unregister()
        self.listener.node.destroy_node()
        self.destroy_node()

    def navigate(self, name, point):
        """Navigate directly to the recorded waypoint."""
        return self.navigate_direct(name, point)

    def navigate_direct(self, name, point):
        """Drive to `point` and return a NavResult; Ctrl+C cancels the goal."""
        try:
            self.buffer.lookup_transform(
                MAP_FRAME, BASE_FRAME, Time(), timeout=Duration(seconds=1.0))
        except tf2_ros.TransformException as e:
            return NavResult(
                'NOT_READY', f'車子還沒定位（{MAP_FRAME} -> {BASE_FRAME}：{e}），請先在 RViz 設定初始位置')
        # goToPose 找不到 action server 會無限等待下去，所以先自己確認
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            return NavResult('NOT_READY', '找不到 Nav2（navigate_to_pose），請先啟動 mecanum_navigation navigation.launch.py')
        if getattr(self, 'abort_requested', lambda: False)():
            return NavResult('CANCELED', '操作手中止任務')
        goal = pose_from_point(point, MAP_FRAME, self.get_clock().now().to_msg())
        if name == 'prep':
            tree = os.path.join(get_package_share_directory('mecanum_navigation'), 'config', 'navigate_prep.xml')
            accepted = self.goToPose(goal, behavior_tree=tree)
        elif name in ('a', 'b'):
            tree = os.path.join(get_package_share_directory('mecanum_navigation'), 'config', 'navigate_transit.xml')
            accepted = self.goToPose(goal, behavior_tree=tree)
        else:
            accepted = self.goToPose(goal)
        if not accepted:
            return NavResult('FAILED', f'Nav2 拒絕了前往 {name} 的目標')
        started = time.monotonic()
        last_print = 0.0
        try:
            while not self.isTaskComplete():
                if getattr(self, 'abort_requested', lambda: False)():
                    self.cancelTask()
                    seconds = time.monotonic() - started
                    return NavResult('CANCELED', arrival_report(name, 'CANCELED', seconds), seconds)
                feedback = self.getFeedback()
                if feedback and time.monotonic() - last_print >= 1.0:
                    last_print = time.monotonic()
                    print(f'\r前往 {name}：還有 {feedback.distance_remaining:.2f} m   ',
                          end='', flush=True)
        except KeyboardInterrupt:
            self.cancelTask()
            getattr(self, 'on_interrupt', lambda: None)()
            print()
            seconds = time.monotonic() - started
            return NavResult('CANCELED', arrival_report(name, 'CANCELED', seconds), seconds)
        print()
        seconds = time.monotonic() - started
        outcome = self.getResult().name
        return NavResult(outcome, arrival_report(name, outcome, seconds), seconds)


def run(node, points, ask=input, say=print, sleep=time.sleep, prepare_start=None, start_before_table=False, select_table=False):
    """Run the mission with the points that are set; return the exit code."""
    unset = unset_points(points)
    ready = [name for name in points if name not in unset]
    if not ready:
        say('目前沒有點位，請先用 wp_record 記錄')
        return 1
    mission = Mission(
        lambda name: node.navigate(name, points[name]), ask, say, sleep, STOP_SECONDS, ready, prepare_start, start_before_table)
    try:
        if select_table:
            say('已載入點位：' + '、'.join(ready))
            mission.run_rounds(select_first=True)
        else:
            mission.run()
    except MissionFinished as e:
        say(str(e))
        return MISSION_FINISHED
    except MissionAborted as e:
        say(f'任務結束：{e}')
    return 0


def main():
    import fcntl
    lock = open('/tmp/mecanum_mission_control.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('已有任務在執行，請先結束原本的終端機或 GUI 任務。', flush=True)
        return 1
    # Ctrl+C 要讓 Python 收到，這樣才能取消正在跑的導航目標
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    from terminal_io import TerminalOperator
    node = Navigator()
    operator = None
    try:
        operator = TerminalOperator()
        node.abort_requested = operator.stop_event.is_set
        node.on_interrupt = operator.stop_event.set
        def interruptible_sleep(seconds):
            if operator.stop_event.wait(seconds):
                raise MissionAborted('操作手中止')
        while True:
            result = run(node, all_points(recorded_only=True), ask=operator.ask,
                         sleep=interruptible_sleep, select_table=True)
            if result == MISSION_FINISHED:
                continue
            if not operator.restart_requested():
                return result
    except KeyboardInterrupt:
        print('\n已結束')
        return 0
    finally:
        if operator is not None:
            operator.close()
        node.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
