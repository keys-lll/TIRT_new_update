#!/usr/bin/env python3
"""Desktop operator interface for the single-file delivery mission."""
import queue
import os
import signal
import subprocess
import tempfile
import json
import uuid
import threading
import time
import fcntl
import sys
import re
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from tf2_ros import TransformException
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
import cv2
import run_mission
from terminal_io import CameraStream


class MissionWindow:
    def __init__(self, root, preview=False):
        self.root = root
        self.preview = preview
        self.closing = False
        self.selected_index = 0
        self.last_submit = 0.0
        self.events = queue.Queue()
        self.camera_stream = CameraStream(self.events)
        self.answers = queue.Queue()
        self.stop_event = threading.Event()
        self.waiting = False
        self.scanning = False
        self.scan_kind = None
        self.scan_started = None
        self.current_extra = '尚未掃描'
        self.phase = '任務準備'
        self.last_message = ''
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.bridge = CvBridge()
        self.root.title('比賽任務')
        self.root.configure(bg='#f5f3ef')
        x, y, sw, sh = 0, 0, self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        try:
            import gi
            gi.require_version('Gdk', '3.0')
            from gi.repository import Gdk
            Gdk.init([])
            display = Gdk.Display.get_default()
            area = (display.get_primary_monitor() or display.get_monitor(0)).get_workarea()
            x, y, sw, sh = area.x, area.y, area.width, area.height
        except (ImportError, ValueError, AttributeError):
            pass
        width, height = min(1100, sw - 140), min(680, sh - 120)
        self.root.geometry(f'{width}x{height}+{x + (sw - width) // 2}+{y + (sh - height) // 2}')
        self.root.resizable(False, False)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        font = 'Noto Sans CJK TC'
        header = tk.Frame(root, bg='#f5f3ef', padx=12, pady=5)
        header.pack(fill='x')
        tk.Label(header, text='送餐機器人', font=(font, 16, 'bold'), fg='#23352e', bg='#f5f3ef').pack(side='left')
        tk.Label(header, text='數字鍵操作' + ('｜介面預覽' if preview else ''), font=(font, 10), fg='#64716a', bg='#f5f3ef').pack(side='left', padx=12)
        tk.Label(header, text='0 中止 ｜ 6 確認／繼續 ｜ 所有鍵位見右側',
                 font=(font, 10), bg='#f5f3ef', fg='#41664d').pack(side='right')
        controls = tk.Frame(root, bg='#f5f3ef', padx=12, pady=6)
        controls.pack(side='right', fill='y')
        tk.Label(controls, text='數字鍵操作', font=(font, 11, 'bold'), bg='#f5f3ef', fg='#2d6b52').grid(row=0, column=0, sticky='ew', pady=(0, 10))
        self.controls = controls
        body = tk.Frame(root, bg='#f5f3ef', padx=12, pady=5)
        body.pack(fill='both', expand=True)
        body.grid_columnconfigure(0, weight=4, uniform='panels')
        body.grid_columnconfigure(1, weight=4, uniform='panels')
        body.grid_columnconfigure(2, weight=3, uniform='panels')
        body.grid_rowconfigure(0, weight=1)
        self.card = tk.Frame(body, bg='#ffffff', padx=12, pady=10)
        self.card.grid(row=0, column=0, sticky='nsew', padx=(0, 8))
        wrap = max(145, int((width - 180) * .34) - 35)
        tk.Label(self.card, text='目前流程', font=(font, 10, 'bold'), fg='#2d6b52', bg='#ffffff').pack(anchor='w')
        self.status = tk.Label(self.card, text='正在載入任務…', font=(font, 15, 'bold'), fg='#23352e', bg='#ffffff', wraplength=wrap, justify='left', anchor='w')
        self.status.pack(fill='x', pady=(4, 8))
        self.detail = tk.Label(self.card, text='', font=(font, 10), fg='#4c5c54', bg='#ffffff', wraplength=wrap, justify='left', anchor='w')
        self.detail.pack(fill='x')
        camera = tk.Frame(body, bg='#ffffff', padx=10, pady=8)
        camera.grid(row=0, column=1, sticky='nsew', padx=(0, 8))
        tk.Label(camera, text='相機串流｜180°', font=(font, 10, 'bold'), fg='#2d6b52', bg='#ffffff').pack(anchor='w')
        self.camera_label = tk.Label(camera, text='預覽模式，不啟動相機' if preview else '正在啟動相機…', font=(font, 10), fg='#64716a', bg='#f5f3ef', wraplength=max(140, int((width - 180) * .34) - 35))
        self.camera_label.pack(fill='both', expand=True, pady=(5, 0))
        self.photo = None
        side = tk.Frame(body, bg='#eaf0e8', padx=10, pady=8)
        side.grid(row=0, column=2, sticky='nsew')
        tk.Label(side, text='備餐區必經路線', font=(font, 10, 'bold'), fg='#2d6b52', bg='#eaf0e8').pack(anchor='w')
        tk.Label(side, text='進：B → A → prep\n出：prep → A → B → 桌號', font=(font, 10), fg='#23352e', bg='#eaf0e8', justify='left').pack(anchor='w', pady=(4, 8))
        tk.Label(side, text='加點餐點', font=(font, 9), fg='#64716a', bg='#eaf0e8').pack(anchor='w')
        side_wrap = max(130, int((width - 180) * .26) - 30)
        self.extra_label = tk.Label(side, text=self.current_extra, font=(font, 13, 'bold'), fg='#795012', bg='#eaf0e8', wraplength=side_wrap, justify='left', anchor='w')
        self.extra_label.pack(fill='x', pady=(2, 6))
        tk.Label(side, text='QR 辨識結果', font=(font, 9), fg='#64716a', bg='#eaf0e8').pack(anchor='w')
        self.qr_label = tk.Label(side, text='等待桌邊掃碼…', font=(font, 12, 'bold'), fg='#795012', bg='#eaf0e8', wraplength=side_wrap, justify='left', anchor='w')
        self.qr_label.pack(fill='x', pady=2)
        self.keymap = {}
        self.buttons = []
        self.choices = []
        self.prompt_id = None
        self.publish_state = None
        self.last_state = 0.0
        self.restart = None
        self.root.bind('<Return>', self.on_enter)
        self.root.bind('<KP_Enter>', self.on_enter)
        self.root.bind('<KeyPress-q>', self.on_abort)
        self.root.bind('<KeyPress-Q>', self.on_abort)
        self.root.bind('<KeyPress>', self.on_keypress)
        for direction in ('Up', 'Down', 'Left', 'Right'):
            self.root.bind('<' + direction + '>', self.on_arrow)
        if not preview:
            self.camera_stream.set_active(True)
        self.root.after(50, self.drain)

    def say(self, text):
        self.events.put(('say', str(text)))

    def ask(self, prompt):
        if self.stop_event.is_set():
            return 'q'
        self.events.put(('ask', (prompt, time.monotonic())))
        answer = self.answers.get()
        return 'q' if self.stop_event.is_set() else answer

    @staticmethod
    def qr_prompt_kind(prompt):
        """Identify the three table-side QR handoffs."""
        for marker, kind in (
            ('請裁判給點餐卡', '點餐卡'),
            ('請裁判給加點卡', '加點卡'),
            ('請裁判給「不須加點」卡', '不須加點卡'),
        ):
            if marker in prompt:
                return kind
        return None

    def qr(self, text):
        if self.waiting and self.scanning:
            self.events.put(('qr', (time.monotonic(), text)))

    def frame(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            image = cv2.rotate(image, cv2.ROTATE_180)
            with self.frame_lock:
                self.latest_frame = image
        except Exception:
            pass

    def show_buttons(self, prompt):
        for button in self.buttons:
            button.destroy()
        self.buttons = []
        if prompt == '__restart__':
            choices = [('重新開始任務', '__restart__'), ('關閉介面', '__close__')]
        elif any(marker in prompt for marker in ('要去哪一桌', '第一回合抽到幾桌', '第二回合抽到幾桌')):
            available = re.search(r'(?:已記錄桌號|可選|只能) ([1-5、]+)', prompt)
            numbers = available.group(1).split('、') if available else list('12345')
            choices = [(f'[{n}] {n} 號桌', n) for n in numbers]
            if '第二回合' in prompt:
                choices.append(('結束回合', ''))
        elif '中繼點未到達' in prompt:
            choices = [('重試', 'r')]
        elif '這一步沒成功' in prompt:
            choices = [('重試', 'r'), ('已手動到位', 's')]
        elif '要去哪些點' in prompt:
            choices = [('開始比賽', '')]
        elif self.scanning:
            choices = [('手動放行', '')]
        else:
            choices = [('確認／繼續', '')]
        if prompt != '__restart__':
            choices.append(('結束任務', 'q'))
        self.choices = choices
        self.keymap = {}
        self.prompt_id = uuid.uuid4().hex
        for index, (label, value) in enumerate(choices):
            if value in ('1', '2', '3', '4', '5'):
                key = value
                label = value + ' 號桌'
            elif value in ('q', '__close__'):
                key = '0'
            elif value == 's':
                key = '7'
            elif value == '' and '第二回合' in prompt:
                key = '8'
            else:
                key = '6'
            assert key not in self.keymap, 'Duplicate numeric shortcut: ' + key
            self.keymap[key] = value
            button = tk.Button(self.controls, text=f'[{key}] {label}', command=lambda v=value: self.activate_choice(v),
                               font=('Noto Sans CJK TC', 12, 'bold'), bg='#dcebdc' if value not in ('q', '__close__') else '#8c4b43',
                               fg='#233b2b' if value not in ('q', '__close__') else 'white', relief='flat', padx=10, pady=11)
            button.grid(row=index + 1, column=0, sticky='ew', pady=(0, 8))
            self.controls.grid_columnconfigure(0, weight=1)
            self.buttons.append(button)
        self.selected_index = 0
        self.highlight_selection()

    def activate_choice(self, value):
        if value == '__close__':
            self.close()
        elif value == 'q':
            self.on_abort()
        else:
            self.submit(value)

    def on_digit(self, value):
        if value == '0':
            if any(answer == '__close__' for _, answer in self.choices):
                self.close()
            else:
                self.on_abort()
        elif self.waiting and value in self.keymap:
            self.activate_choice(self.keymap[value])
        return 'break'

    def highlight_selection(self):
        for i, button in enumerate(self.buttons):
            abort = self.choices[i][1] in ('q', '__close__')
            button.config(bg='#e9b95f' if i == self.selected_index else ('#8c4b43' if abort else '#dcebdc'),
                          fg='#233b2b' if i == self.selected_index or not abort else 'white')
        if self.buttons:
            self.buttons[self.selected_index].focus_set()

    def on_arrow(self, event):
        if self.waiting and self.buttons:
            step = {'Left': -1, 'Right': 1, 'Up': -1, 'Down': 1}[event.keysym]
            self.selected_index = (self.selected_index + step) % len(self.buttons)
            self.highlight_selection()
        return 'break'

    def on_keypress(self, event):
        value = getattr(event, 'char', '').translate(str.maketrans('０１２３４５６７８９', '0123456789'))
        keypad = {'KP_Insert': '0', 'KP_Right': '6', 'KP_Home': '7', 'KP_Up': '8', 'KP_Prior': '9', 'KP_End': '1', 'KP_Down': '2', 'KP_Next': '3',
                  'KP_Left': '4', 'KP_Begin': '5'}
        if not value:
            value = keypad.get(event.keysym, event.keysym.removeprefix('KP_'))
        if value in '0123456789' and len(value) == 1:
            return self.on_digit(value)

    def terminal_key(self, value):
        if value.lower() == 'q':
            self.on_abort()
        elif value in '0123456789' and len(value) == 1:
            self.on_digit(value)
        elif self.waiting and value.lower() in ('r', 's') and any(
                answer == value.lower() for label, answer in self.choices):
            self.submit(value.lower())
        elif not value:
            self.on_enter()

    def on_table_key(self, value):
        """Select a table only when its choice is available in this prompt."""
        if self.waiting and any(answer == value for label, answer in self.choices):
            self.submit(value)
        return 'break'

    def on_abort(self, event=None):
        """End the mission, including a goal currently being navigated."""
        if any(value == '__restart__' for _, value in self.choices):
            return 'break'
        self.stop_event.set()
        self.detail.config(text='正在中止任務，取消導航…')
        if self.waiting:
            self.submit('q')
        return 'break'

    def on_enter(self, event=None):
        """Confirm a prompt, or activate the button selected with Tab."""
        if not self.waiting:
            return 'break'
        focused = self.root.focus_get()
        for button in self.buttons:
            if focused is button:
                button.invoke()
                return 'break'
        primary = [(label, value) for label, value in self.choices if value != 'q']
        # Multiple choices (tables / retry) must be selected explicitly.
        if len(primary) == 1:
            self.submit(primary[0][1])
        return 'break'

    def submit(self, value=None):
        if not self.waiting:
            return
        answer = '' if value is None else value
        if answer != 'q' and time.monotonic() - self.last_submit < 0.25:
            return
        self.last_submit = time.monotonic()
        if self.scan_kind == '加點卡' and answer.strip().lower() not in ('q', 'quit'):
            self.current_extra = answer.strip() or '未輸入'
            self.extra_label.config(text=self.current_extra)
        self.waiting = False
        self.scanning = False
        with self.frame_lock:
            self.latest_frame = None
        self.photo = None
        self.camera_label.config(image='', text='相機持續運作…')
        self.scan_kind = None
        self.scan_started = None
        self.prompt_id = None
        self.choices = []
        self.keymap = {}
        for button in self.buttons:
            button.destroy()
        self.buttons = []
        if answer == '__restart__' and self.restart:
            self.phase = '任務準備'
            self.current_extra = '尚未掃描'
            self.extra_label.config(text=self.current_extra)
            self.status.config(text='正在重新開始…')
            self.detail.config(text='')
            self.restart()
        else:
            self.answers.put(answer)

    def close(self):
        if self.preview:
            self.root.destroy()
            return
        if self.closing:
            return
        if any(value == '__close__' for _, value in self.choices):
            self.root.destroy()
            return
        self.closing = True
        self.on_abort()
        self.detail.config(text='正在取消導航並關閉任務…')

    def drain(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'say':
                    if value.startswith('====='):
                        self.phase = value.strip('= ')
                        self.status.config(text=self.phase)
                        if '第一趟' in value:
                            self.current_extra = '尚未掃描'
                            self.extra_label.config(text=self.current_extra)
                    else:
                        self.last_message = value
                        self.detail.config(text=value)
                        if value.startswith(('正在前往 ', '正在初始化定位')):
                            self.status.config(text=self.phase + '｜' + value)
                elif kind == 'ask':
                    value, requested_at = value
                    value = value.replace('Enter', '6').replace('q 結束', '0 中止').replace('[r]', '[6]').replace('[s]', '[7]').replace('[q]', '[0]')
                    if '第二回合' in value and '抽到幾桌' in value:
                        value = value.replace('直接按 6', '直接按 8')
                    self.waiting = True
                    self.scan_kind = self.qr_prompt_kind(value)
                    self.scanning = self.scan_kind is not None
                    self.scan_started = requested_at if self.scanning else None
                    if self.scanning and not self.preview:
                        self.camera_stream.set_active(True)
                    if self.scanning:
                        with self.frame_lock:
                            self.latest_frame = None
                        self.qr_label.config(text='等待本次掃描…')
                        self.camera_label.config(image='', text='正在啟動相機／QR…')
                    self.status.config(text=self.phase + ('｜等待 QR Code：' + self.scan_kind if self.scanning else '｜等待操作'))
                    if '要去哪些點' in value:
                        self.detail.config(text='選桌後直接出發；定位沿用導航啟動設定。')
                    elif '這一步沒成功' in value or '中繼點未到達' in value:
                        self.status.config(text=self.phase + '｜導航未完成')
                        self.detail.config(text=self.last_message + '\n\n' + value)
                    else:
                        self.detail.config(text=(value + '\n請掃描' + self.scan_kind + ' QR Code；或按 6 手動放行。\n未掃到且未按 6，會持續停留等待。' if self.scanning else (value + '\n可按數字 1～5 選桌' if '抽到幾桌' in value or '要去哪一桌' in value else value)))
                    self.show_buttons(value)
                    if self.stop_event.is_set():
                        self.submit('q')
                elif kind == 'qr':
                    observed_at, code = value
                    if code.strip():
                        self.qr_label.config(text=code.strip())
                        if self.waiting and self.scanning and observed_at >= self.scan_started:
                            kind = self.scan_kind
                            self.submit(code.strip())
                            self.detail.config(text='已掃描' + kind + '：' + code.strip())
                elif kind == 'camera_error':
                    self.camera_label.config(image='', text=value)
                elif kind == 'keyboard':
                    self.terminal_key(value)
                elif kind == 'remote':
                    try:
                        command = json.loads(value)
                        valid = [item[1] for item in self.choices]
                        if self.waiting and command.get('prompt_id') == self.prompt_id and command.get('value') in valid:
                            self.submit(command['value'])
                    except (ValueError, TypeError, AttributeError):
                        pass
                elif kind == 'done':
                    if self.closing:
                        self.root.destroy()
                        return
                    self.scanning = False
                    self.waiting = False
                    for button in self.buttons:
                        button.destroy()
                    self.buttons = []
                    self.status.config(text='任務已結束')
                    self.detail.config(text=value)
                    self.waiting = True
                    self.show_buttons('__restart__')
                    self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        except queue.Empty:
            pass
        with self.frame_lock:
            frame = self.latest_frame
            self.latest_frame = None
        if frame is not None:
            width = max(1, self.camera_label.winfo_width())
            height = max(1, self.camera_label.winfo_height())
            if width > 1 and height > 1:
                image = PILImage.fromarray(frame)
                image.thumbnail((width, height))
                self.photo = ImageTk.PhotoImage(image)
                self.camera_label.config(image=self.photo, text='')
        if self.publish_state and time.monotonic() - self.last_state >= 0.5:
            self.last_state = time.monotonic()
            self.publish_state(json.dumps({'status': self.status.cget('text'), 'detail': self.detail.cget('text'),
                'extra': self.extra_label.cget('text'), 'qr': self.qr_label.cget('text'),
                'prompt_id': self.prompt_id, 'choices': self.choices}, ensure_ascii=False))
        self.root.after(50, self.drain)


def main():
    if '--preview' in sys.argv:
        root = tk.Tk()
        app = MissionWindow(root, preview=True)
        app.events.put(('ask', ('要去哪一桌？（已記錄桌號 1、2、3、4、5；q 結束）> ', time.monotonic())))
        root.after(200, root.focus_force)
        try:
            root.mainloop()
        finally:
            app.camera_stream.close()
        return 0
    lock = open('/tmp/mecanum_mission_control.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('已有送餐介面在執行，請先結束原本的任務。', file=sys.stderr)
        return 1
    root = tk.Tk()
    app = MissionWindow(root)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    qr_node = rclpy.create_node('mission_gui_qr')
    state_pub = qr_node.create_publisher(String, '/car_mission/state', 10)
    def publish_state(text):
        msg = String()
        msg.data = text
        state_pub.publish(msg)
    app.publish_state = publish_state
    qr_node.create_subscription(String, '/car_mission/command', lambda msg: app.events.put(('remote', msg.data)), 10)
    qr_node.create_subscription(String, '/zbar_opencv_code', lambda msg: app.qr(msg.data), 10)
    qr_node.create_subscription(Image, '/zbar_opencv', app.frame, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(qr_node)
    qr_thread = threading.Thread(target=executor.spin, daemon=True)
    qr_thread.start()

    def worker():
        node = None
        try:
            node = run_mission.Navigator()
            node.abort_requested = app.stop_event.is_set
            node.on_interrupt = app.stop_event.set
            def navigate(name, point):
                app.say('正在前往 ' + name + '…')
                if app.stop_event.is_set():
                    raise run_mission.MissionAborted('按 Q 中止')
                result = node.navigate(name, point)
                if app.stop_event.is_set():
                    raise run_mission.MissionAborted('按 Q 中止')
                return result
            node.navigate_for_gui = navigate
            class Adapter:
                def navigate(self, name, point):
                    return node.navigate_for_gui(name, point)
            def interruptible_sleep(seconds):
                if app.stop_event.wait(seconds):
                    raise run_mission.MissionAborted('按 Q 中止')
            while not app.stop_event.is_set():
                points = run_mission.all_points(recorded_only=True)
                outcome = run_mission.run(Adapter(), points, app.ask, app.say,
                                          interruptible_sleep, select_table=True)
                if outcome != run_mission.MISSION_FINISHED:
                    break
            result = '可按重新開始任務，或關閉視窗。'
        except Exception as exc:
            result = '程式發生錯誤：' + str(exc)
        finally:
            if node is not None:
                node.close()
        app.events.put(('done', result))
    def start_worker():
        while not app.answers.empty():
            app.answers.get_nowait()
        app.stop_event.clear()
        app.root.protocol('WM_DELETE_WINDOW', app.close)
        app.worker_thread = threading.Thread(target=worker, daemon=True)
        app.worker_thread.start()
    app.restart = start_worker
    start_worker()
    def terminal_keyboard():
        print('鍵盤操作：桌號 1～5 後按 6；桌邊掃 QR 或 6 手動放行；備餐區 Enter 出發；其他步驟 Enter 下一步；q 後按 6 中止。', flush=True)
        for line in sys.stdin:
            app.events.put(('keyboard', line.strip()))
    if sys.stdin.isatty():
        threading.Thread(target=terminal_keyboard, daemon=True).start()
    try:
        root.mainloop()
    finally:
        app.stop_event.set()
        app.answers.put('q')
        app.worker_thread.join(timeout=5)
        app.camera_stream.close()
        executor.shutdown()
        qr_thread.join(timeout=3)
        qr_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
