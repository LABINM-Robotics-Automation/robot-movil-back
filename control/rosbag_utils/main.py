import subprocess
import sys
import time

ROBOT_MOVIL_IP = '192.168.0.10'

def ping_ip(ip):
    """Ping an IP address to check if it exists in the network."""
    try:
        # Use the ping command to check the connectivity
        result = subprocess.run(['ping', '-c', '1', ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return result.returncode == 0  # Return True if ping is successful (return code 0)
    except Exception as e:
        print(f"Error pinging IP {ip}: {e}")
        return False


# SE USA
def execute_bash(command, wait=True):
    """Execute a bash command."""
    try:
        if wait:
            # Blocking version (wait for the command to finish)
            result = subprocess.run(command, shell=True, check=True, text=True, capture_output=True)
            print("Output:", result.stdout)  # Print the standard output
            print("Error:", result.stderr)    # Print any error messages if present
            return result.returncode == 0  # Return True if successful (exit code 0), otherwise False
        else:
            # Non-blocking version (run command in the background)
            process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            print(f"Command started: {command}")
            return process  # Return the process object to check later if needed
    except subprocess.CalledProcessError as e:
        print("An error occurred while executing the command:", e)
        print("Return code:", e.returncode)
        print("Output:", e.output)
        return False


# SE USA
def record_bagfile(topics, date, test_number, description, duration=None):
    """
    Records a ROS bag file for the specified topics and assigns a static node name.
    
    Parameters:
    - topics: List of ROS topics to record (e.g., ['/topic1', '/topic2'])
    - date: Date for bag file naming (e.g., '24-10-12')
    - test_number: The test number for the naming convention.
    - description: A short description for the test (e.g., 'detection-test').
    - duration: Optional duration in seconds for the recording to stop automatically.
    
    Returns:
    - process: The Popen process object to control the recording.
    """
    bagfile_name = f"{description}_{test_number}_{date}.bag" 
    topics_str = ' '.join(topics)    
    duration_option = f"--duration={duration}" if duration else "" 
    command = f"rosbag record -O {bagfile_name} {topics_str} {duration_option} __name:=rosbag_record" 
    print(f"Recording bagfile with command: {command}")
    
    process = subprocess.Popen(command, 
                               shell=True, 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True) 
    return process


# SE USA
def play_bagfile(bagfile_name, loop=False, rate=1.0, start_time=0.0, duration=None):
    """
    Plays a ROS bag file using rosbag play.

    Parameters:
    - bagfile_name: The name or full path to the .bag file to play.
    - loop: Boolean flag to enable loop playback (default is False).
    - rate: Playback rate (default is 1.0 for real-time).
    - start_time: Time in seconds to start the playback (default is 0.0, start from the beginning).
    - duration: If provided, play only for this duration in seconds.
    
    Returns:
    - process: The Popen process object, so the playback can be controlled (stopped, etc.).
    """
    # Base rosbag play command
    command = f"rosbag play {bagfile_name}"
    
    # Add options based on function arguments
    if loop:
        command += " --loop"
    
    if rate != 1.0:
        command += f" --rate {rate}"
    
    if start_time > 0.0:
        command += f" --start {start_time}"
    
    if duration:
        command += f" --duration {duration}"

    print(f"Playing bagfile with command: {command}")
    
    # Start the command in the background
    process = subprocess.Popen(command, 
                               shell=True, 
                               stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, 
                               text=True)
    
    return process  # Return the process object to control it later if needed
