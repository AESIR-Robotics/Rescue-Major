#!/usr/bin/env python3
"""
Keyboard teleoperation node for JointControl messages.
Works over SSH / headless terminals using termios raw mode.

Keybindings (configurable via ROS2 params):
  Each joint gets two keys: one to increment, one to decrement.
  Joints are shown in the live HUD rendered in the terminal.
  
Velocity control:
  ↑ / ↓ : Increase / Decrease global velocity
  v     : Toggle velocity mode (constant / zero on release)

Usage:
  ros2 run <your_pkg> joint_mux.py
  ros2 run <your_pkg> joint_mux.py --ros-args -p step:=0.05 -p topic:=/joint_cmd
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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult


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
VEL_UP_KEY = '+'         # Up arrow
VEL_DOWN_KEY = '-'       # Down arrow
VEL_MODE_KEY = 'v'            # Toggle velocity mode


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

    def __init__(self):
        super().__init__('joint_keyboard_teleop')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('topic',       'hardware_node/joint_command')
        self.declare_parameter('step',        0.1)       # position step per keypress (rad / m)
        self.declare_parameter('speed_step',  0.1)       # velocity adjustment step
        self.declare_parameter('min_speed',   0.0)
        self.declare_parameter('max_speed',   2.0)
        self.declare_parameter('joint_names', list(DEFAULT_JOINT_KEYS.keys()))
        self.declare_parameter('frame_id',    'world')

        self.topic_       = self.get_parameter('topic').value
        self.step_        = self.get_parameter('step').value
        self.speed_step_  = self.get_parameter('speed_step').value
        self.min_speed_   = self.get_parameter('min_speed').value
        self.max_speed_   = self.get_parameter('max_speed').value
        self.frame_id_    = self.get_parameter('frame_id').value
        joint_names_      = self.get_parameter('joint_names').value
        self.speed_       = 0.1

        # ── Velocity control state ──────────────────────────────────────────
        self.velocity_mode_ = 'constant'  # 'constant' or 'hold'
        self.active_joints_ = set()       # joints currently being pressed

        # ── Parameter callbacks ─────────────────────────────────────────────
        self.add_on_set_parameters_callback(self._parameter_callback)

        # ── Build key → (joint_index, direction) map ────────────────────────
        # Uses DEFAULT_JOINT_KEYS for joints that appear there,
        # auto-assigns keys for any extras not in the default map.
        extra_key_pool = list('uijkop')   # fallback keys for extra joints
        self.joints_    = list(joint_names_)
        self.positions_ = [0.0] * len(self.joints_)
        self.velocities_= [0.0] * len(self.joints_)
        self.accelerations_ = [20000.0] * len(self.joints_)
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

    # ── Parameter callback ────────────────────────────────────────────────────
    def _parameter_callback(self, params):
        """Handle dynamic parameter updates (topic, step, speed, joint_names)."""
        
        result_success = True
        
        for param in params:
            if param.name == 'topic':
                old_topic = self.topic_
                self.topic_ = param.value
                # Recreate publisher with new topic
                self.destroy_publisher(self.pub_)
                self.pub_ = self.create_publisher(JointControl, self.topic_, 10)
                self.get_logger().info(f'Topic changed: {old_topic} → {self.topic_}')
                
            elif param.name == 'step':
                if param.value > 0.0:
                    self.step_ = param.value
                    self.get_logger().info(f'Step changed to {self.step_:.4f} rad')
                else:
                    self.get_logger().warn(f'Invalid step {param.value}, must be > 0')
                    result_success = False

            elif param.name == 'joint_names':
                # Rebuild joint structure
                old_joints = self.joints_
                self.joints_ = list(param.value)
                
                # Preserve positions where joint names match
                new_positions = [0.0] * len(self.joints_)
                new_velocities = [0.0] * len(self.joints_)
                new_efforts = [0.0] * len(self.joints_)
                
                for new_idx, new_name in enumerate(self.joints_):
                    if new_name in old_joints:
                        old_idx = old_joints.index(new_name)
                        new_positions[new_idx] = self.positions_[old_idx]
                        new_velocities[new_idx] = self.velocities_[old_idx]
                        new_efforts[new_idx] = self.efforts_[old_idx]
                
                self.positions_ = new_positions
                self.velocities_ = new_velocities
                self.efforts_ = new_efforts
                
                # Rebuild key map
                self.key_map_ = {}
                self.bindings_ = {}
                extra_key_pool = list('uijkop')
                
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
                
                self.get_logger().info(f'Joint names updated: {self.joints_}')
        
        return SetParametersResult(successful=result_success)


    def _get_key(self, timeout: float = 0.1) -> str:
        """Read one key without blocking longer than timeout seconds."""
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
                # Handle escape sequences (arrows etc.)
                if key == '\x1b':
                    rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if rlist2:
                        seq = sys.stdin.read(2)   # consume '[A' etc.
                        return '\x1b' + seq
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
        msg.acceleration    = list(self.accelerations_)
        msg.effort          = list(self.efforts_)
        return msg


    def _render_hud(self):
        lines = []
        lines.append(clr(BOLD + CYAN,
            '╔══════════════════════════════════════════════════════╗'))
        lines.append(clr(BOLD + CYAN,
            '║     Joint Keyboard Teleop  (ROS2)                    ║'))
        lines.append(clr(BOLD + CYAN,
            '╚══════════════════════════════════════════════════════╝'))
        lines.append(f'  Topic : {clr(YELLOW, self.topic_)}')
        
        # Velocity info with mode indicator
        mode_indicator = clr(GREEN, '●') if self.velocity_mode_ == 'constant' else clr(YELLOW, '○')
        lines.append(
            f'  Step  : {clr(YELLOW, f"{self.step_:.4f}")} rad   '
            f'Speed : {clr(YELLOW, f"{self.speed_:.4f}")} rad/s {mode_indicator}   '
            f'Range : {clr(DIM, f"[{self.min_speed_:.1f}, {self.max_speed_:.1f}]")}'
        )
        lines.append(f'  Mode  : {clr(GREEN if self.velocity_mode_ == "constant" else YELLOW, self.velocity_mode_.upper())}')
        lines.append('')
        lines.append(clr(BOLD, f'  {"Joint":<20} {"Inc":>4}  {"Dec":>4}   {"Position":>10}  {"Vel":>7}'))
        lines.append('  ' + '─' * 58)

        for idx, name in enumerate(self.joints_):
            inc_k, dec_k = self.bindings_[name]
            pos_str = f'{self.positions_[idx]:+.4f}'
            vel_str = f'{self.velocities_[idx]:+.3f}'
            
            # Normalize to [0, 1] range for bar display
            bar_val = self.positions_[idx] / (2 * math.pi)
            bar_len = 12
            filled  = int(bar_val * bar_len)
            bar = '┼' + '█' * filled + '─' * (bar_len - filled)
            
            # Color based on activity
            is_active = idx in self.active_joints_
            pos_color = GREEN if self.positions_[idx] != 0.0 else DIM
            vel_color = RED if is_active else (GREEN if self.velocities_[idx] != 0.0 else DIM)
            
            lines.append(
                f'  {name:<20} '
                f'{clr(GREEN, "[" + inc_k + "]"):>4}  '
                f'{clr(RED,   "[" + dec_k + "]"):>4}   '
                f'{clr(pos_color, pos_str):>10}  '
                f'{clr(DIM, bar)}  '
                f'{clr(vel_color, vel_str):>7}'
            )

        lines.append('')
        lines.append(clr(BOLD, '  Velocity Control:'))
        lines.append(f'  {clr(GREEN, "[↑]")} Increase speed   '
                     f'{clr(RED, "[↓]")} Decrease speed   '
                     f'{clr(YELLOW, "[v]")} Toggle mode')
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
                    self.velocities_ = [0.0] * len(self.joints_)
                    self.efforts_    = [0.0] * len(self.joints_)
                    self.active_joints_.clear()
                    self.last_key_   = key
                    self.pub_.publish(self._build_msg())

                elif key == ZERO_VEL_EFFORT_KEY:
                    self.velocities_ = [0.0] * len(self.joints_)
                    self.efforts_    = [0.0] * len(self.joints_)
                    self.active_joints_.clear()
                    self.last_key_   = key
                    self.pub_.publish(self._build_msg())

                elif key == VEL_UP_KEY:
                    self.speed_ = min(self.max_speed_, self.speed_ + self.speed_step_)
                    self.last_key_ = '↑'

                elif key == VEL_DOWN_KEY:
                    self.speed_ = max(self.min_speed_, self.speed_ - self.speed_step_)
                    self.last_key_ = '↓'

                elif key == VEL_MODE_KEY:
                    # Toggle velocity mode
                    self.velocity_mode_ = 'hold' if self.velocity_mode_ == 'constant' else 'constant'
                    self.last_key_ = key

                    if self.velocity_mode_ == 'hold':
                        # Mandar todas las posiciones a -1
                        self.positions_ = [-1.0] * len(self.joints_)
                    else:
                        # Restaurar posiciones válidas
                        self.positions_ = [0.0] * len(self.joints_)
                        self.velocities_ = [0.0] * len(self.joints_)
                        self.active_joints_.clear()

                    self.pub_.publish(self._build_msg())

                elif key in self.key_map_:
                    idx, direction = self.key_map_[key]

                    if self.velocity_mode_ == 'constant':
                        # Modo normal (posición incremental)
                        new_pos = self.positions_[idx] + direction * self.step_
                        self.positions_[idx] = new_pos % (2 * math.pi)
                        self.velocities_[idx] = direction * self.speed_
                        self.active_joints_.add(idx)

                    else:  # HOLD MODE
                        current_vel = self.velocities_[idx]

                        # Si ya se mueve en dirección opuesta → detener
                        if current_vel * direction < 0:
                            self.velocities_[idx] = 0.0
                            if idx in self.active_joints_:
                                self.active_joints_.remove(idx)

                        else:
                            # Aplicar velocidad en dirección indicada
                            self.velocities_[idx] = direction * self.speed_
                            self.active_joints_.add(idx)

                    self.last_key_ = key
                    self.pub_.publish(self._build_msg())

                elif key:
                    self.last_key_ = key

                self._render_hud()
                rclpy.spin_once(self, timeout_sec=0.0)

        finally:
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