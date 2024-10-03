import subprocess

def get_pid_with_command(command):
    result = subprocess.run(['ps', 'aux'], stdout=subprocess.PIPE, text=True)

    for line in result.stdout.splitlines():
        if command in line:
            columns = line.split()
            pid = int(columns[1])
            return pid
    return None

command = '/opt/ros/humble/lib/rclcpp_components/component_container_isolated'
pid = get_pid_with_command(command)

print(pid)
