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
import tempfile


# def execute_script(script, verbose=True):
#     """
#     Executes a shell script passed as a string.
#
#     Args:
#         script (str): The shell script content.
#         verbose (bool): Whether to print debug and output information.
#
#     Returns:
#         tuple: (stdout, stderr) output of the script execution.
#
#     Raises:
#         RuntimeError: If the script execution fails.
#     """
#     temp_script_path = None
#     try:
#         # Create a temporary file to hold the script
#         with tempfile.NamedTemporaryFile(delete=False, suffix=".sh") as temp_script:
#             temp_script.write(script.encode())
#             temp_script_path = temp_script.name
#
#         # Make the script executable
#         os.chmod(temp_script_path, 0o755)
#
#         if verbose:
#             print(f"Temporary script path: {temp_script_path}")
#
#         result = subprocess.run(
#             ["bash", temp_script_path],
#             check=True,
#             text=True,
#             capture_output=True,
#             executable="/bin/bash"
#         )
#
#         print("STDOUT:", result.stdout)
#         print("STDERR:", result.stderr)
#
#         if verbose:
#             print("Script output:", result.stdout)
#             if result.stderr:
#                 print("Script errors:", result.stderr)
#
#         return result.stdout, result.stderr
#
#     except subprocess.CalledProcessError as e:
#         if verbose:
#             print("Error while running the script:", e.stderr)
#         raise RuntimeError(f"Script execution failed with return code {e.returncode}") from e
#     except Exception as e:
#         raise RuntimeError(f"An unexpected error occurred while executing the script: {e}") from e
#     finally:
#         # Cleanup: Delete the temporary script file
#         if temp_script_path and os.path.exists(temp_script_path):
#             os.remove(temp_script_path)
#             if verbose:
#                 print(f"Deleted temporary script: {temp_script_path}")
#

# def execute_script(script):
#     """
#     Execute a shell script passed as a string.
#
#     Args:
#         script (str): The shell script content.
#     """
#     import subprocess
#     try:
#         # Execute the script via bash
#         result = subprocess.run(
#             script,
#             shell=True,
#             check=True,
#             text=True,
#             capture_output=True,
#             executable="/bin/bash"  # Ensure it runs in bash
#         )
#         print("Script output:")
#         print(result.stdout)
#         if result.stderr:
#             print("Script errors:")
#             print(result.stderr)
#     except subprocess.CalledProcessError as e:
#         print(f"Error while executing the script:\n{e.stderr}")
#

import libtmux

def run_on_tmux_session(session_name, command, server):

    # Connect to the tmux server

    # Check if the session already exists
    session = server.find_where({"session_name": session_name})
    if session:
        print(f"Tmux session '{session_name}' already exists.")
        return

    # Create a new session
    session = server.new_session(session_name=session_name, kill_session=True, attach=False)
    window = session.attached_window
    pane = window.attached_pane

    # Reset environment variables to use default Python paths
    pane.send_keys("export PATH=/usr/bin:/bin:/usr/sbin:/sbin")
    pane.send_keys("unset PYTHONPATH")
    pane.send_keys("noetic")
    pane.send_keys("source ~/.bashrc")
    pane.send_keys(command)  # Run the desired command

    print(f"Tmux session '{session_name}' created and command sent.")


def stop_tmux_session(session_name, server):
        session = server.find_where({"session_name": session_name})
        if session:
            session.kill_session()
            print(f"Tmux session '{session_name}' killed successfully.")
        else:
            print(f"Tmux session '{session_name}' not found.")
        return

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
            process = subprocess.Popen(
                command, 
                shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True
            )
            print(f"Command started: {command}")
            return process  # Return the process object to check later if needed
    except subprocess.CalledProcessError as e:
        print("An error occurred while executing the command:", e)
        print("Return code:", e.returncode)
        print("Output:", e.output)
        return False


# SE USA
from datetime import datetime
import os

def record_bagfile(topics, name, basedir, duration=None):

    filepath = os.path.join(basedir, name)

    bagfile_name = f"{filepath}" 
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
