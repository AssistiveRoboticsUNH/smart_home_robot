import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import Float32, Int64, Int32, Bool
from datetime import datetime
import os, re, time, asyncio, subprocess, threading

from simple_logger.discord_bot import DiscordNotifier

# ===== Env config (set these in ~/.bashrc and source before running) =====
# export DISCORD_TOKEN="..."
# export DISCORD_LOGGING_CHANNEL=123...
# export DISCORD_EMERGENCY_CHANNEL=456...
# export ROBOT_NAME="Galivant"
# export robot_pass="your-sudo-password"
DISCORD_TOKEN             = os.getenv("DISCORD_TOKEN", "")
DISCORD_LOGGING_CHANNEL   = int(os.getenv("DISCORD_LOGGING_CHANNEL", "0"))
DISCORD_EMERGENCY_CHANNEL = int(os.getenv("DISCORD_EMERGENCY_CHANNEL", "0"))
ROBOT_NAME                = os.getenv("ROBOT_NAME", "Robot")
ROBOT_PASS                = os.getenv("robot_pass") or os.getenv("ROBOT_PASS", "")

# ===== Local log folder =====
BASE_LOG_DIR = os.path.join(os.path.expanduser("~"), "shr_logs", "discord_logs")
os.makedirs(BASE_LOG_DIR, exist_ok=True)

class LogSubscriber(Node):

    def __init__(self):
        super().__init__('log_rosout')
        self.subscription = self.create_subscription(Log, 'rosout', self.listener_callback, 10)
        self.exclude_list = ["ZeroMQ", "/home", "high_level_domain_Idle", "starting undocking"]
        
        self.bump_subscriber = self.create_subscription(Int64, 'bump', self.bump_callback, 10)
        self.voltage_subscriber = self.create_subscription(Float32, 'charging_voltage', self.voltage_callback, 10)
        # self.ir_sensor_subscriber = self.create_subscription(Float32, 'docking/ir_weight', self.ir_sensor_callback, 10)
        self.charger_subscriber = self.create_subscription(Bool, 'charging', self.iot_charger_callback, 10)
        self.current_subscriber = self.create_subscription(Float32, 'charging_current', self.current_callback, 10)
        
        self.charger_status = None
        self.bump = None
        self.voltage = None

        self.notified_start = False
        self.prev_info = ""

        self.isDiscord_connected = False
        try:
            if not DISCORD_TOKEN or not DISCORD_LOGGING_CHANNEL or not DISCORD_EMERGENCY_CHANNEL:
                raise RuntimeError("Missing env vars: DISCORD_TOKEN, DISCORD_LOGGING_CHANNEL, DISCORD_EMERGENCY_CHANNEL")
            self.notifier = DiscordNotifier(token=DISCORD_TOKEN, channel_id=DISCORD_LOGGING_CHANNEL, on_message_callback=self.on_message_callback)
            self.emergency_notifier = DiscordNotifier(token=DISCORD_TOKEN, channel_id=DISCORD_EMERGENCY_CHANNEL)
            self.isDiscord_connected = True
            self.get_logger().info(f'Discord connection established')
        except Exception as e:
            self.get_logger().warn(f'Error in discord connection:{e}')
            self.get_logger().warn(f'Will log offline')
            self.notifier = None
            self.emergency_notifier = None

        # --- Watchdog using a monotonic deadline ---
        self.watch_trigger_phrase = "starting ros"   # lower
        self.watch_success_phrase = "running match"  # lower
        self.watch_timeout_sec = 30 * 60  # 30 minutes
        self.deadline_monotonic = None
        self.alert_sent = False
        self.watchdog_timer = self.create_timer(5.0, self._watchdog_tick)

        # --- Dock failure emergency phrase (lower) ---
        self.dock_fail_phrase = "robot failed to dock"
    
    # ---------- Telemetry callbacks ----------
    def iot_charger_callback(self, msg):
        self.charger_status = msg.data

    def bump_callback(self, msg):
        self.bump = msg.data

    def voltage_callback(self, msg):
        self.voltage = msg.data

    def current_callback(self, msg):
        self.current = msg.data

    # ---------- File logging ----------
    def log_offline(self, info):
        '''
        append log text in loacl file
        '''
        # Generate the filename based on the current date
        date_str = datetime.now().strftime("Y%y_M%m_D%d")
        self.simple_log_file = f"{BASE_LOG_DIR}/log_{date_str}.txt"

        
        with open(self.simple_log_file, 'a+') as f:
            f.write(info)

    # ---------- Discord helpers ----------
    async  def on_message_callback(self, msg):
        # print(f"Received :{msg}")
        msg = (msg or "").strip().lower()
        self.get_logger().info(f'Received :{msg}')
        time_str = datetime.now().strftime('%m/%d/%Y  %H:%M:%S')
        if msg == "help":
            available_commands = "**Available Commands:**\nstatus \nrunstop\nrun\nreboot\n"
            await self.notifier.send_message(f'{available_commands}')
        elif msg == "status":
            status = f"Charging: {self.charger_status},\nBump: {self.bump},\nVoltage: {self.voltage:.2f} V, \nCurrent: {self.current:.2f} Amps"
            self.get_logger().info(f'Robot Status :{status}\n')
            await self.notifier.send_message(f'{time_str} >> Robot Status: \n{status}')
            # asyncio.create_task(notifier.send_message(status))
        
        elif msg == "runstop":
            command = "ros2 service call /runstop std_srvs/srv/SetBool \"{data: true}\""
            result = await self.execute_cmd(command)
            await self.notifier.send_message(f'{result}')

        elif msg == "run":
            command = "ros2 service call /runstop std_srvs/srv/SetBool \"{data: false}\""
            result = await self.execute_cmd(command)
            await self.notifier.send_message(f'{result}')

        elif msg == "stop_lidar":
            command = "ros2 service call /stop_motor std_srvs/srv/Empty"
            result = await self.execute_cmd(command)
            await self.notifier.send_message(f'{result}')
            
        elif msg == "start_lidar":
            command = "ros2 service call /start_motor std_srvs/srv/Empty"
            result = await self.execute_cmd(command)
            await self.notifier.send_message(f'{result}')

        elif msg == "reboot":
            if not ROBOT_PASS:
                await self.notifier.send_message("Reboot requested, but 'robot_pass' is not set in environment.")
                return
            await self.notifier.send_message(f"Reboot command received. Rebooting now...")
            try:
                subprocess.run(
                    ["sudo", "-S", "-p", "", "reboot"],
                    input=ROBOT_PASS + "\n",
                    text=True,
                    capture_output=True,
                    timeout=3.0,   # return quickly; system will reboot
                )
            except subprocess.TimeoutExpired:
                pass  # reboot proceeding
            except Exception:
                await self.notifier.send_message(f"Reboot command failed.")
            return


    async def execute_cmd(self, command: str, timeout=5.0):
        try:
            result = subprocess.run(
                command, shell=True, text=True, capture_output=True,
                timeout= timeout
            )
            out = result.stdout.strip() or result.stderr.strip() or "No output"
        except subprocess.TimeoutExpired:
            out = "No output"
        except Exception:
            out = "No output"

        result = f"```bash\n$ {command}\n\n{out}\n```"
        return result


    # ---------- Watchdog utilities ----------
    def _start_watchdog(self):
        self.deadline_monotonic = time.monotonic() + self.watch_timeout_sec
        self.alert_sent = False
        self.get_logger().info("Emergency watchdog started (30 min).")

    def _clear_watchdog(self):
        if self.deadline_monotonic is not None:
            self.get_logger().info("Emergency watchdog cleared by success phrase.")
        self.deadline_monotonic = None
        self.alert_sent = False

    async def _watchdog_tick(self):
        if self.deadline_monotonic is None or self.alert_sent:
            return
        if time.monotonic() >= self.deadline_monotonic:
            self.alert_sent = True
            when_str = datetime.now().strftime('%m/%d/%Y  %H:%M:%S')
            alert = f"{ROBOT_NAME}: A protocol didn't complete properly, please check ASAP. )."
            # schedule async send on our private loop
            await self.emergency_notifier.send_message(f"**{alert}**")
            self.deadline_monotonic = None


    async def listener_callback(self, msg):
        
        # print('info:', msg) 
        stamp=msg.stamp
        name=msg.name
        file=msg.file 
        data=msg.msg 
        data_l = data.lower()
        td=datetime.fromtimestamp(stamp.sec).strftime("%m/%d/%Y  %H:%M:%S")


        if not self.notified_start:
            await self.notifier.send_message(f'{td} >> **Robot Started**\n')
            self.log_offline(f'\n{td} >> **Robot Started**\n')
            self.notified_start = True

        
        # --- Emergency: explicit dock failure phrase (case-insensitive) ---
        if self.dock_fail_phrase.lower() in data_l:
            await self.emergency_notifier.send_message(f"🚨🚨🚨 **Emergency** 🚨🚨🚨 \n{ROBOT_NAME}: Failed to dock! Please check ASAP")

        # Watchdog triggers from raw stream (case-insensitive)
        if self.watch_trigger_phrase in data_l:
            self._start_watchdog()
        if self.watch_success_phrase in data_l:
            self._clear_watchdog()

        magic_key='weblog='
        if data_l.startswith(magic_key):
            data=data.replace(magic_key,'')
            
            for word in self.exclude_list:
                if word in data:
                    return

            # Remove timestamp if present at the beginning
            cleaned_text = re.sub(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}', '', data)
                
            info=f'{td} >> {cleaned_text}\n'
            
            # Dont send same message consecutively
            if self.prev_info == cleaned_text:
                return

            self.get_logger().info(f'>> {info}')

            if self.isDiscord_connected:
                await self.notifier.send_message(info)

            # Log offline as well
            self.log_offline(info)

            
            self.prev_info = cleaned_text
            



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = LogSubscriber() 
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()