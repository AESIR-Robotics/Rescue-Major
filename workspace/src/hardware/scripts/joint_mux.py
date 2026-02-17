#!/usr/bin/env python3
"""
Keyboard teleoperation node for JointControl messages.
Works over SSH / headless terminals using termios raw mode.

Keybindings (configurable via ROS2 params):
  Each joint gets two keys: one to increment, one to decrement.
  Joints are shown in the live HUD rendered in the terminal.

Usage:
  ros2 run <your_pkg> joint_keyboard_teleop
  ros2 run <your_pkg> joint_keyboard_teleop --ros-args -p step:=0.05 -p topic:=/joint_cmd
"""

import sys
import os
import select
import termios
import tty
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


from hardware.msg import JointControl  


# ── Default joint configuration ───────────────────────────────────────────────
# Format: { 'joint_name': ('inc_key', 'dec_key') }
# Add or remove joints as needed.
DEFAULT_JOINT_KEYS = {
    'flipper_0': ('q', 'a'),
    'flipper_1': ('w', 's'),
    'flipper_2': ('e', 'd'),
    'flipper_3': ('r', 'f'),
    'flipper_4': ('t', 'g'),
    'flipper_5': ('y', 'h'),
}

# Keys that are always active
QUIT_KEYS = ('\x03', '\x1b')   # Ctrl+C, Escape
RESET_KEY = 'z'
ZERO_VEL_EFFORT_KEY = 'x'     # zero out velocity and effort fields



CLEAR      = '\033[2J\033[H'
BOLD       = '\033[1m'
DIM        = '\033[2m'
GREEN      = '\033[92m'
CYAN       = '\033[96m'
YELLOW     = '\033[93m'
RED        = '\033[91m'
RESET_CLR  = '\033[0m'
MOVE_TOP   = '\033[H'


def clr(code, text):
    return f'{code}{text}{RESET_CLR}'


# ── Node ──────────────────────────────────────────────────────────────────────
class JointKeyboardTeleop(Node):

    speedDef = 1.0

    def __init__(self):
        super().__init__('joint_keyboard_teleop')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('topic',       'hardware_node/joint_command')
        self.declare_parameter('step',        0.1)       # position step per keypress (rad / m)
        self.declare_parameter('joint_names', list(DEFAULT_JOINT_KEYS.keys()))
        self.declare_parameter('frame_id',    'world')

        self.topic_     = self.get_parameter('topic').value
        self.step_      = self.get_parameter('step').value
        self.frame_id_  = self.get_parameter('frame_id').value
        joint_names_    = self.get_parameter('joint_names').value

        # ── Build key → (joint_index, direction) map ────────────────────────
        # Uses DEFAULT_JOINT_KEYS for joints that appear there,
        # auto-assigns keys for any extras not in the default map.
        extra_key_pool = list('uijkop')   # fallback keys for extra joints
        self.joints_    = list(joint_names_)
        self.positions_ = [0.0] * len(self.joints_)
        self.velocities_= [JointKeyboardTeleop.speedDef] * len(self.joints_)
        self.efforts_   = [0.0] * len(self.joints_)

        self.key_map_   = {}   # key -> (joint_idx, delta)
        self.bindings_  = {}   # joint_name -> (inc_key, dec_key)  for display

        for idx, name in enumerate(self.joints_):
            if name in DEFAULT_JOINT_KEYS:
                inc_k, dec_k = DEFAULT_JOINT_KEYS[name]
            elif extra_key_pool:
                inc_k = extra_key_pool.pop(0)
                dec_k = extra_key_pool.pop(0) if extra_key_pool else '?'
            else:
                inc_k, dec_k = '?', '?'

            self.bindings_[name] = (inc_k, dec_k)
            if inc_k != '?':
                self.key_map_[inc_k] = (idx, +1)
            if dec_k != '?':
                self.key_map_[dec_k] = (idx, -1)


        self.pub_ = self.create_publisher(JointControl, self.topic_, 10)


        self.settings_  = termios.tcgetattr(sys.stdin)
        self.last_key_  = ''
        self.running_   = True


    def _get_key(self, timeout: float = 0.1) -> str:
        """Read one key without blocking longer than timeout seconds."""
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
                # Handle escape sequences (arrows etc.) — consume but ignore
                if key == '\x1b':
                    rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if rlist2:
                        sys.stdin.read(2)   # consume '[A' etc.
                    return '\x1b'
                return key
            return ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)


    def _build_msg(self) -> JointControl:
        msg = JointControl()
        now = self.get_clock().now().to_msg()
        msg.header = Header()
        msg.header.stamp    = now
        msg.header.frame_id = self.frame_id_
        msg.joint_names     = list(self.joints_)
        msg.position        = list(self.positions_)
        msg.velocity        = list(self.velocities_)
        msg.effort          = list(self.efforts_)
        return msg


    def _render_hud(self):
        lines = []
        lines.append(clr(BOLD + CYAN,
            '╔══════════════════════════════════════════╗'))
        lines.append(clr(BOLD + CYAN,
            '║     Joint Keyboard Teleop  (ROS2)        ║'))
        lines.append(clr(BOLD + CYAN,
            '╚══════════════════════════════════════════╝'))
        lines.append(f'  Topic : {clr(YELLOW, self.topic_)}   '
                     f'Step : {clr(YELLOW, str(self.step_))} rad/m')
        lines.append('')
        lines.append(clr(BOLD, f'  {"Joint":<20} {"Inc":>4}  {"Dec":>4}   {"Position":>10}'))
        lines.append('  ' + '─' * 46)

        for idx, name in enumerate(self.joints_):
            inc_k, dec_k = self.bindings_[name]
            pos_str = f'{self.positions_[idx]:+.4f}'
            bar_val = max(0, min(1.0, self.positions_[idx]))
            bar_len = 15
            filled  = int(abs(bar_val) * bar_len)
            if bar_val >= 0:
                bar = '┼' + '█' * filled + ' ' * (bar_len - filled)
            else:
                bar = ' ' * (bar_len - filled) + '█' * filled + '┼' 
            color = GREEN if self.positions_[idx] != 0.0 else DIM
            lines.append(
                f'  {name:<20} '
                f'{clr(GREEN, "[" + inc_k + "]"):>4}  '
                f'{clr(RED,   "[" + dec_k + "]"):>4}   '
                f'{clr(color, pos_str):>10}  '
                f'{clr(DIM, bar)}'
            )

        lines.append('')
        lines.append(clr(DIM, f'  [{RESET_KEY}] Reset all   '
                              f'[{ZERO_VEL_EFFORT_KEY}] Zero vel/effort   '
                              f'[ESC/Ctrl+C] Quit'))
        lines.append('')
        key_display = repr(self.last_key_) if self.last_key_ else '─'
        lines.append(f'  Last key: {clr(YELLOW, key_display)}')

        # Move cursor to top and overwrite (avoids flicker vs. full clear)
        sys.stdout.write(MOVE_TOP)
        sys.stdout.write('\n'.join(lines))
        sys.stdout.write('\033[J')   # clear from cursor to end of screen
        sys.stdout.flush()

    def run(self):
        print(CLEAR, end='')

        try:
            while rclpy.ok() and self.running_:
                key = self._get_key(timeout=0.05)

                if key in QUIT_KEYS:
                    self.running_ = False
                    break

                elif key == RESET_KEY:
                    self.positions_  = [0.0] * len(self.joints_)
                    self.velocities_ = [JointKeyboardTeleop.speedDef] * len(self.joints_)
                    self.efforts_    = [0.0] * len(self.joints_)
                    self.last_key_   = key
                    self.pub_.publish(self._build_msg())

                elif key == ZERO_VEL_EFFORT_KEY:
                    self.velocities_ = [0.0] * len(self.joints_)
                    self.efforts_    = [0.0] * len(self.joints_)
                    self.last_key_   = key

                elif key in self.key_map_:
                    idx, direction = self.key_map_[key]
                    if(0 < self.positions_[idx] + direction * self.step_ < math.pi * 2):
                        self.positions_[idx] += direction * self.step_
                        self.velocities_[idx] = direction * JointKeyboardTeleop.speedDef
                    self.last_key_ = key
                    self.pub_.publish(self._build_msg())

                elif key:
                    self.last_key_ = key   # show unknown key in HUD

                self._render_hud()
                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
            # Always restore terminal, even on crash
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)
            print('\n\n' + clr(YELLOW, 'Terminal restored. Bye!') + '\n')



def main(args=None):
    rclpy.init(args=args)
    node = JointKeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()