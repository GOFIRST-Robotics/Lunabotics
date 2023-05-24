from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import re


def generate_launch_description():
  def get_target_ip(target: str, default: str | None):
    try:
      nmap = subprocess.Popen(
          ('nmap', '-sn', '192.168.1.1/24'), stdout=subprocess.PIPE)
      grep = subprocess.check_output(('grep', target), stdin=nmap.stdout)
      nmap.wait()
      res = grep.decode()
      return re.sub(r'.*\((.*)\).*', r'\g<1>', res) or default
    except:
      return default

  ld = LaunchDescription()

  rovr_control = Node(
      package="rovr_control",
      executable="main_control_node",
      parameters=[{'ip': get_target_ip('blixt-G14', '192.168.1.117')}],
  )

  motor_control = Node(
      package="motor_control",
      executable="motor_control_node"
  )

  ld.add_action(rovr_control)
  ld.add_action(motor_control)

  return ld
