#!/usr/bin/env python3
"""Dual-mode keyboard teleoperation: Joint Control + Twist Control"""

import sys, select, termios, tty, math, time, os, signal, subprocess, socket
from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from hardware.msg import JointControl
from collections import deque

# Terminal codes
CLEAR, BOLD, DIM = '\033[2J\033[H', '\033[1m', '\033[2m'
GREEN, CYAN, YELLOW, RED, MAGENTA = '\033[92m', '\033[96m', '\033[93m', '\033[91m', '\033[95m'
RESET_CLR, MOVE_TOP = '\033[0m', '\033[H'
def clr(c, t): return f'{c}{t}{RESET_CLR}'

class KeyboardReader:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self, timeout=0.05):
        tty.setraw(sys.stdin.fileno())
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if not rlist: return ''
            ch = sys.stdin.read(1)
            if ch == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        return f'\x1b[{sys.stdin.read(1)}'
                    return ch + ch2
                return ch
            return ch
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def restore(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

class TeleopInterface(ABC):
    def __init__(self, node):
        self.node, self.last_key, self.active = node, '', False
    
    @abstractmethod
    def handle_key(self, key): pass
    @abstractmethod
    def render(self): pass
    @abstractmethod
    def on_activate(self): pass
    @abstractmethod
    def on_deactivate(self): pass
    @abstractmethod
    def stop(self): pass
    @abstractmethod
    def update(self): pass
    

DEFAULT_JOINT_KEYS = {
    'flipper_0': ('q', 'a'), 'flipper_1': ('w', 's'), 'flipper_2': ('e', 'd'),
    'flipper_3': ('r', 'f'), 
    'joint_1': ('t', 'g'), 'joint_2': ('y', 'h'), 'joint_3': ('u', 'j'), 
    'joint_4': ('i', 'k'), 'joint_5': ('o', 'l'), 'joint_6': ('n', 'm'), 'joint_7': ('v', 'b')
}

class JointControlInterface(TeleopInterface):
    def __init__(self, node, topic, joint_names):
        super().__init__(node)
        self.topic, self.joint_names = topic, joint_names
        self.step, self.speed, self.speed_step = 0.1, 0.5, 0.1
        self.min_speed, self.max_speed = 0.0, 2.0
        self.velocity_mode = 'hold'

        self.backupConstant = [0.0] * len(joint_names)
        self.backupHold = [-1.0] * len(joint_names)

        self.positions = self.backupHold

        self.velocities = [0.0] * len(joint_names)
        self.efforts = [0.0] * len(joint_names)
        self.key_map, self.bindings = {}, {}
        self._build_key_map()
        self.pub = node.create_publisher(JointControl, topic, 10)
        self.republish_rate, self.last_publish_time = 10.0, time.time()
    
    def _build_key_map(self):
        extra = list('tgyhujikolpmn')
        for idx, name in enumerate(self.joint_names):
            if name in DEFAULT_JOINT_KEYS:
                inc_k, dec_k = DEFAULT_JOINT_KEYS[name]
            elif extra:
                inc_k, dec_k = extra.pop(0), (extra.pop(0) if extra else '?')
            else:
                inc_k, dec_k = '?', '?'
            self.bindings[name] = (inc_k, dec_k)
            if inc_k != '?': self.key_map[inc_k] = (idx, +1)
            if dec_k != '?': self.key_map[dec_k] = (idx, -1)
    
    def on_activate(self):
        self.last_publish_time = time.time()
    
    def on_deactivate(self):
        self.stop()
    
    def handle_key(self, key):
        self.last_key = key
        if key == '+':
            self.speed = min(self.max_speed, self.speed + self.speed_step)
            self._update_active_velocities()
            self._publish()
            return True
        elif key == '-':
            self.speed = max(self.min_speed, self.speed - self.speed_step)
            self._update_active_velocities()
            self._publish()
            return True
        elif key == 'c':
            self.velocity_mode = 'hold' if self.velocity_mode == 'constant' else 'constant'
            if self.velocity_mode == "hold":
                self.positions = self.backupHold
                self.stop()
            else:
                self.positions = self.backupConstant
                self._publish()
            return True
        elif key == 'z':
            self.positions = [-1.0 if self.velocity_mode == 'hold' else 0.0] * len(self.joint_names)
            self.stop()
            return True
        elif key == 'x':
            self.stop()
            return True
        elif key in self.key_map:
            idx, direction = self.key_map[key]
            if self.velocity_mode == 'constant':
                self.positions[idx] = (self.positions[idx] + direction * self.step) % (2 * math.pi)
                self.velocities[idx] = direction * self.speed
            else:

                self.positions[idx] = -1.0 
                if direction * self.velocities[idx] < 0:
                    self.velocities[idx] = 0.0
                else:
                    self.velocities[idx] = direction * self.speed
                
            self._publish()
            return True
        return False
    
    def _update_active_velocities(self):
        for idx, vel in enumerate(self.velocities):
            if vel != 0:
                self.velocities[idx] = self.speed * (-1 if vel < 0 else 1)
    
    def stop(self):
        self.velocities, self.efforts = [0.0] * len(self.joint_names), [0.0] * len(self.joint_names)
        self._publish()

    def update(self):
        current_time = time.time()
        if current_time - self.last_publish_time >= 1.0 / self.republish_rate:
            if any(v != 0.0 for v in self.velocities):
                self._publish()
    
    def _publish(self):
        msg = JointControl()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.joint_names, msg.position = list(self.joint_names), list(self.positions)
        msg.velocity, msg.effort = list(self.velocities), list(self.efforts)
        msg.acceleration = [math.pi / 2] * len(self.joint_names)
        self.pub.publish(msg)
        self.last_publish_time = time.time()
    
    def render(self):
        lines = [
            clr(BOLD + CYAN, '╔══════════════════════════════════════════════════════╗'),
            clr(BOLD + CYAN, '║         [1] JOINT CONTROL MODE                       ║'),
            clr(BOLD + CYAN, '╚══════════════════════════════════════════════════════╝'),
            f'  Topic : {clr(YELLOW, self.topic)}',
        ]
        mode_color = GREEN if self.velocity_mode == 'constant' else MAGENTA
        mode_symbol = '●' if self.velocity_mode == 'constant' else '◐'
        lines.append(f'  Step: {clr(YELLOW, f"{self.step:.4f}")} rad   Speed: {clr(YELLOW, f"{self.speed:.4f}")} rad/s {clr(mode_color, mode_symbol)}   Mode: {clr(mode_color, self.velocity_mode.upper())}')
        if self.velocity_mode == 'hold':
            lines.append(f'  {clr(MAGENTA, "HOLD MODE: Position = -1 (no target), velocity only while pressed")}')
        lines.extend(['', clr(BOLD, f'  {"Joint":<20} {"Inc":>4}  {"Dec":>4}   {"Position":>10}  {"Vel":>7}'), '  ' + '─' * 58])
        for idx, name in enumerate(self.joint_names):
            inc_k, dec_k = self.bindings[name]
            pos_val, vel_val = self.positions[idx], self.velocities[idx]
            pos_str = clr(MAGENTA, '  (hold)') if pos_val == -1.0 else f'{pos_val:+.4f}'
            vel_str = f'{vel_val:+.3f}'
            if self.velocity_mode == 'constant' and pos_val >= 0:
                bar_val, bar_len = pos_val / (2 * math.pi), 12
                filled = int(bar_val * bar_len)
                bar = '┼' + '█' * filled + '─' * (bar_len - filled)
            else:
                bar = ' ' * 13
            pos_color = GREEN if pos_val > 0.0 and pos_val != -1.0 else DIM
            vel_color = RED if vel_val != 0.0 else DIM
            lines.append(f'  {name:<20} {clr(GREEN, "["+inc_k+"]"):>4}  {clr(RED, "["+dec_k+"]"):>4}   {clr(pos_color, pos_str):>10}  {clr(DIM, bar)}  {clr(vel_color, vel_str):>7}')
        lines.extend(['', clr(BOLD, '  Controls:'),
            f'  {clr(GREEN, "[↑]")} ↑ Speed   {clr(RED, "[↓]")} ↓ Speed   {clr(YELLOW, "[v]")} Toggle mode   {clr(DIM, "[z]")} Reset   {clr(DIM, "[x]")} Stop',
            f'  {clr(CYAN, "[2]")} Switch to Twist mode', '', f'  Last key: {clr(YELLOW, repr(self.last_key) if self.last_key else "─")}'])
        return '\n'.join(lines)

class TwistControlInterface(TeleopInterface):
    MOVE_BINDINGS = {
        'i': (1,0,0,0), 'o': (1,0,0,1), 'j': (0,0,0,-1), 'l': (0,0,0,1),
        'u': (1,0,0,-1), ',': (-1,0,0,0), '.': (-1,0,0,1), 'm': (-1,0,0,-1), 'k': (0,0,0,0),
    }
    
    def __init__(self, node, topic):
        super().__init__(node)
        self.topic, self.speed, self.turn = topic, 0.5, 1.0
        self.speed_step, self.turn_step = 0.1, 0.1
        self.min_speed, self.max_speed = 0.0, 2.0
        self.min_turn, self.max_turn = 0.0, 3.0
        self.x, self.y, self.z, self.th = 0, 0, 0, 0
        self.pub = node.create_publisher(Twist, topic, 10)
        self.republish_rate, self.last_publish_time = 10.0, time.time()
    
    def on_activate(self):
        self.stop()
        self.last_publish_time = time.time()
    
    def on_deactivate(self):
        self.stop()
    
    def handle_key(self, key):
        self.last_key = key
        if key in self.MOVE_BINDINGS:
            self.x, self.y, self.z, self.th = self.MOVE_BINDINGS[key]
            self.th *= -1
            self._publish()
            return True
        elif key == 'q':
            self.speed = min(self.max_speed, self.speed + self.speed_step)
            self._publish()
            return True
        elif key == 'z':
            self.speed = max(self.min_speed, self.speed - self.speed_step)
            self._publish()
            return True
        elif key == 'w':
            self.turn = min(self.max_turn, self.turn + self.turn_step)
            self._publish()
            return True
        elif key == 'x':
            self.turn = max(self.min_turn, self.turn - self.turn_step)
            self._publish()
            return True
        elif key == 'k' or key == ' ':
            self.stop()
            return True
        return False
    
    def stop(self):
        self.x = self.y = self.z = self.th = 0
        self._publish()
    
    def update(self):
        current_time = time.time()
        if current_time - self.last_publish_time >= 1.0 / self.republish_rate:
            if self.x != 0 or self.y != 0 or self.z != 0 or self.th != 0:
                self._publish()
    
    def _publish(self):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = self.x * self.speed, self.y * self.speed, self.z * self.speed
        msg.angular.x, msg.angular.y, msg.angular.z = 0.0, 0.0, self.th * self.turn 
        self.pub.publish(msg)
        self.last_publish_time = time.time()
    
    def render(self):
        linear, angular = self.x * self.speed, self.th * self.turn
        return '\n'.join([
            clr(BOLD + GREEN, '╔══════════════════════════════════════════════════════╗'),
            clr(BOLD + GREEN, '║         [2] TWIST CONTROL MODE                       ║'),
            clr(BOLD + GREEN, '╚══════════════════════════════════════════════════════╝'),
            f'  Topic : {clr(YELLOW, self.topic)}',
            f'  Linear Speed: {clr(YELLOW, f"{self.speed:.2f}")} m/s   Angular Speed: {clr(YELLOW, f"{self.turn:.2f}")} rad/s',
            '',
            f'  Current: Linear = {clr(CYAN if linear != 0 else DIM, f"{linear:+.2f}")} m/s   Angular = {clr(CYAN if angular != 0 else DIM, f"{angular:+.2f}")} rad/s',
            '',
            clr(BOLD, '  Movement:'),
            f'        {clr(GREEN, "[u]")}    {clr(GREEN, "[i]")}    {clr(GREEN, "[o]")}',
            f'        {clr(DIM, " ↰      ↑      ↱ ")}',
            '',
            f'        {clr(YELLOW, "[j]")}    {clr(RED, "[k]")}    {clr(YELLOW, "[l]")}',
            f'        {clr(DIM, " ←    STOP    → ")}',
            '',
            f'        {clr(GREEN, "[m]")}    {clr(GREEN, "[,]")}    {clr(GREEN, "[.]")}',
            f'        {clr(DIM, " ↲      ↓      ↳ ")}',
            '',
            clr(BOLD, '  Speed Controls:'),
            f'  {clr(GREEN, "[q]")} Linear ↑   {clr(RED, "[z]")} Linear ↓   {clr(YELLOW, "[w]")} Angular ↑   {clr(MAGENTA, "[x]")} Angular ↓',
            f'  {clr(CYAN, "[1]")} Switch to Joint mode',
            '',
            f'  Last key: {clr(YELLOW, repr(self.last_key) if self.last_key else "─")}',
        ])

class DualTeleopNode(Node):
    def __init__(self):
        super().__init__('dual_teleop')
        self.declare_parameter('joint_topic', 'hardware_node/joint_command')
        self.declare_parameter('twist_topic', 'hardware_node/cmd_vel')
        self.declare_parameter('joint_names', list(DEFAULT_JOINT_KEYS.keys()))
        joint_topic = self.get_parameter('joint_topic').value
        twist_topic = self.get_parameter('twist_topic').value
        joint_names = self.get_parameter('joint_names').value

        self.joint_interface = JointControlInterface(self, joint_topic, joint_names)
        self.twist_interface = TwistControlInterface(self, twist_topic)
        self.current_interface = self.joint_interface
        self.current_interface.active = True
        self.current_interface.on_activate()
        self.keyboard = KeyboardReader()
        self.running = True

        self.debug_log = deque(maxlen=10)
        
        # Rendering optimization
        self.last_render_content = None
        self.force_full_render = True 

        # --- Deadman state ---
        self.operator_deadman_triggered_ = False
        self.last_key_time = time.time()
        
        # SIGHUP handler 
        signal.signal(signal.SIGHUP, self._handle_sighup)

        self.debug_log.append("Started control interface")
    
    def _handle_sighup(self, signum, frame):
        self.debug("SIGHUP received")
        self.stopInterf()
        self.running = False

    def debug(self, msg):
        t = time.strftime("%H:%M:%S")
        self.debug_log.append(f"[{t}] {msg}")

    def stopInterf(self):
        self.twist_interface.stop()
        self.joint_interface.stop()

    def switch_interface(self, interface):
        if self.current_interface == interface: return
        self.current_interface.on_deactivate()
        self.current_interface.active = False
        self.current_interface = interface
        self.current_interface.active = True
        self.current_interface.on_activate()
        self.force_full_render = True  # Force full re-render on mode switch
    
    def render(self, force_full=False):
        """
        Render interface with incremental updates to reduce flicker.
        Only does full clear/redraw when necessary.
        """
        # Build current frame content
        content_lines = []
        content_lines.append(self.current_interface.render())
        
        # Deadman status indicators
        status_line = '\n\n' + clr(BOLD, 'STATUS: ')
        if self.operator_deadman_triggered_:
            status_line += clr(YELLOW, 'INACTIVITY TIMEOUT')
        else:
            status_line += clr(GREEN, 'Active')
        content_lines.append(status_line)
        
        content_lines.append(GREEN + '\n\nDEBUG LOG:\n' + RESET_CLR)
        content_lines.append(YELLOW + '\n'.join(self.debug_log) + RESET_CLR + '\n')
        
        current_content = ''.join(content_lines)
        
        # Determine if we need full render or can do incremental
        needs_full_render = (
            force_full or 
            self.force_full_render or 
            self.last_render_content is None or
            len(current_content) != len(self.last_render_content)
        )
        
        if needs_full_render:
            # Full render: clear screen and redraw everything
            sys.stdout.write(CLEAR)
            sys.stdout.write(MOVE_TOP)
            sys.stdout.write(current_content)
            sys.stdout.write('\033[J')  # Clear to end of screen
            self.force_full_render = False
        else:
            # Incremental render: only update if content changed
            if current_content != self.last_render_content:
                # Content changed but same length - overwrite in place
                sys.stdout.write(MOVE_TOP)
                sys.stdout.write(current_content)
        
        sys.stdout.flush()
        self.last_render_content = current_content

    def run(self):
        print(CLEAR, end='')
        try:
            self.last_key_time = time.time()
            while rclpy.ok() and self.running:
                # Get key FIRST (highest priority for responsiveness)
                key = self.keyboard.get_key(timeout=0.05)

                # Operator deadman
                if key != '':
                    self.operator_deadman_triggered_ = False
                    self.last_key_time = time.time()

                # Inactivity deadman
                now = time.time()
                if now - self.last_key_time > 3.0 and not self.operator_deadman_triggered_:
                    self.operator_deadman_triggered_=True
                    self.debug("Inactivity detected, stopping everything")
                    self.stopInterf()

                # Handle keys
                if key in ('\x03', '\x1b'):
                    self.running = False
                    break
                elif key == '1':
                    self.switch_interface(self.joint_interface)
                elif key == '2':
                    self.switch_interface(self.twist_interface)
                elif key:
                    self.current_interface.handle_key(key)
                
                self.current_interface.update()
                
                self.render()
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            self.stopInterf()
            self.keyboard.restore()
            print('\n\n' + clr(YELLOW, 'Terminal restored. Bye!') + '\n')

def main(args=None):
    rclpy.init(args=args)
    node = DualTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()