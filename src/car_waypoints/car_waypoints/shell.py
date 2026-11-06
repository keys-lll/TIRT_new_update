"""Terminal commands for wp_record and wp_go; ROS access is injected."""
import math
import re

from car_waypoints.store import load_waypoints, save_waypoints

NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*\Z')
RESERVED = ('list', 'del', 'go', 'init', 'help', 'q', 'quit')
QUIT = ('q', 'quit')
LEG_LIMIT = 90
LEG_LIMIT_WITH_OBSTACLES = 120


def describe(name, wp):
    return f'{name}: x={wp.x:.3f} y={wp.y:.3f} yaw={math.degrees(wp.yaw):.1f}°'


def describe_all(waypoints):
    if not waypoints:
        return '（還沒有任何點）'
    return '\n'.join(describe(name, wp) for name, wp in waypoints.items())


def unknown(name, waypoints):
    return f'沒有 {name}；目前有：{", ".join(waypoints) or "（無）"}'


def not_understood(line):
    return f'看不懂「{line.strip()}」，輸入 help 看說明'


def arrival_report(name, outcome, seconds):
    """Describe a finished goal; outcome is a TaskResult name."""
    if outcome == 'CANCELED':
        return f'已取消前往 {name}（{seconds:.1f} 秒）'
    if outcome != 'SUCCEEDED':
        return f'前往 {name} 失敗（{outcome}，{seconds:.1f} 秒）'
    text = f'到達 {name}，花了 {seconds:.1f} 秒'
    # 簡章：單趟無障礙 90 秒、有障礙 120 秒
    if seconds > LEG_LIMIT_WITH_OBSTACLES:
        return f'{text}  ⚠ 超過有障礙單趟上限 {LEG_LIMIT_WITH_OBSTACLES} 秒'
    if seconds > LEG_LIMIT:
        return f'{text}  ⚠ 超過無障礙單趟上限 {LEG_LIMIT} 秒'
    return text


class RecordShell:
    """Save the robot's current pose under a typed name."""

    HELP = '\n'.join([
        '<名稱>      把車子目前位置存成這個名稱（已存在就覆寫），例如 start、t1、prep',
        'a / A       儲存備餐區前方中繼點 a（不必刻意對準車頭）',
        'b / B       儲存 5 號桌旁中繼點 b（不必刻意對準車頭）',
        'del <名稱>  刪除一個點',
        'list        列出所有點',
        'q           離開',
        f'名稱：英文字母開頭，只用英數字和底線，不能是 {", ".join(RESERVED)}',
    ])

    def __init__(self, path, get_pose, on_change=None):
        self.path = path
        self.get_pose = get_pose
        self.on_change = on_change

    def handle(self, line):
        """Run one command line; return (text to print, keep running)."""
        words = line.split()
        if len(words) == 1 and words[0] in ('A', 'B'):
            words[0] = words[0].lower()
        elif len(words) == 2 and words[0] == 'del' and words[1] in ('A', 'B'):
            words[1] = words[1].lower()
        if not words:
            return '', True
        verb = words[0]
        if verb in QUIT:
            return '', False
        if verb == 'help':
            return self.HELP, True
        try:
            waypoints = load_waypoints(self.path)
        except ValueError as e:
            return f'點位檔有錯：{e}', True
        if words == ['list']:
            return describe_all(waypoints), True
        if verb == 'del' and len(words) == 2:
            name = words[1]
            if name not in waypoints:
                return unknown(name, waypoints), True
            old = waypoints.pop(name)
            self._save(waypoints)
            return f'已刪除 {describe(name, old)}', True
        if len(words) != 1 or not NAME.match(verb) or verb in RESERVED:
            return not_understood(line), True
        try:
            wp = self.get_pose()
        except LookupError as e:
            return f'讀不到目前位置：{e}', True
        old = waypoints.get(verb)
        waypoints[verb] = wp
        self._save(waypoints)
        if old is None:
            return f'已儲存 {describe(verb, wp)}', True
        return f'已覆寫 {describe(verb, wp)}（原本 {describe(verb, old)}）', True

    def _save(self, waypoints):
        save_waypoints(self.path, waypoints)
        if self.on_change:
            self.on_change(waypoints)


class GoShell:
    """Navigate to, or localise at, one named waypoint per command."""

    HELP = '\n'.join([
        'go <名稱>    開到這個點，到了就停下等下一個指令（Ctrl+C 取消）',
        'init <名稱>  告訴 AMCL 車子現在就在這個點（例如 init start）',
        'list         列出所有點（每次都重新讀檔，wp_record 新存的點馬上可用）',
        'q            離開',
    ])

    def __init__(self, path, navigate, set_initial_pose):
        self.path = path
        self.navigate = navigate
        self.set_initial_pose = set_initial_pose

    def handle(self, line):
        """Run one command line; return (text to print, keep running)."""
        words = line.split()
        if not words:
            return '', True
        verb = words[0]
        if verb in QUIT:
            return '', False
        if verb == 'help':
            return self.HELP, True
        try:
            waypoints = load_waypoints(self.path)
        except ValueError as e:
            return f'點位檔有錯：{e}', True
        if words == ['list']:
            return describe_all(waypoints), True
        if verb in ('go', 'init') and len(words) == 2:
            name = words[1]
            if name not in waypoints:
                return unknown(name, waypoints), True
            if verb == 'go':
                return self.navigate(name, waypoints[name]), True
            self.set_initial_pose(waypoints[name])
            return f'已把初始位置設在 {describe(name, waypoints[name])}', True
        if len(words) == 1 and verb in waypoints:
            return f'要出發請輸入 go {verb}', True
        return not_understood(line), True
