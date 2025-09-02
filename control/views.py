# from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework.response import Response
from django.http import FileResponse
from django.http import HttpResponse
from django.http import JsonResponse
# from rest_framework import status
from control.rosbag_utils.main import run_on_tmux_session, stop_tmux_session, execute_bash, record_bagfile, play_bagfile
from django.conf import settings
import os
from datetime import datetime
import os
from django.http import JsonResponse
from .ros_utils.rosbridge_websocket_manager import RosWebsocketManager  
from pathlib import Path
import libtmux

# Import for bag to video conversion (will be available when opencv is installed)
try:
    from control.rosbag_utils.bag_to_video_converter import convert_bag_to_mp4
except ImportError:
    convert_bag_to_mp4 = None


FOLDER_RECORD_PATH = os.path.join(settings.BASE_DIR, 'control', 'videos')
FOLDER_MP4_PATH = os.path.join(settings.BASE_DIR, 'control', 'converted_videos')
PATH_FILE = os.path.join(settings.BASE_DIR, 'control', 'rosbag_utils', 'start-zed2i-camera.sh') 

server = libtmux.Server()



@api_view(['POST'])
def start_camera(request):
    try:
        # camera_process = execute_bash(PATH_FILE, wait=False)
        # execute_bash('/opt/ros/noetic/bin/roslaunch zed_wrapper zed2i.launch', wait=False)

        command = 'roslaunch zed_wrapper zed2i.launch'
        session_name = 'zed-camera'
        run_on_tmux_session(session_name, command, server)

        return Response({'mensaje': "Zed2i iniciada en proceso"}, status=200)

    except Exception as e: 
        return Response({ 'mensaje' : f"Ocurrrió un error: {str(e)}" }, status=500)



@api_view(['POST'])
def stop_camera(request):
    try:
        session_name = 'zed-camera'
        stop_tmux_session(session_name, server)
        return Response({'mensaje': "Zed2i stopped"},status=200)

    except Exception as e: 
        return Response({'mensaje' : f"Ocurrrió un error: {str(e)}"}, status=500)



@api_view(['POST'])
def start_image_processor(request):
    try:
        # command = 'rosrun mobile_robot_iot image2compressedImage.py __name:=image_processor'
        command = 'rosrun blueberry-detection-ros image2compressedImage.py __name:=image_processor'
        session_name = 'image-processor'
        run_on_tmux_session(session_name, command, server)
        return Response({'mensaje': "image processor iniciada en proceso"}, status=200)

    except Exception as e: 
        return Response({ 'mensaje' : f"Ocurrrió un error: {str(e)}" }, status=500)



@api_view(['POST'])
def stop_image_processor(request):
    try:
        session_name = 'image-processor'
        stop_tmux_session(session_name, server)
        return Response({'mensaje': "image processor stopped"},status=200)

    except Exception as e:
        print(e)
        return Response({'mensaje' : f"Ocurrrió un error: {str(e)}"}, status=500)



@api_view(['POST'])
def start_record(request):
    try:

        session_name = 'bagfile-record-thread'
        topics = ['/zed2i/zed_node/rgb_raw/image_raw_color']

        session = server.find_where({"session_name": session_name})
        
        if not(session):
            session = server.new_session(session_name=session_name, kill_session=True, attach=False)

        window = session.attached_window
        pane = window.attached_pane

        pane.send_keys("export PATH=/usr/bin:/bin:/usr/sbin:/sbin")
        pane.send_keys("unset PYTHONPATH")
        # pane.send_keys("noetic")
        pane.send_keys("source ~/.bashrc")


        directory_path = os.path.join(settings.BASE_DIR, 'control', 'videos')
        directory = Path(directory_path)
        directory.mkdir(parents=True, exist_ok=True)
        pane.send_keys(f"cd {directory_path}") 


        date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        bagfile_name = str(date) + '.bag'
        topics_str = ' '.join(topics)
        pane.send_keys(f'rosbag record -O {bagfile_name} {topics_str} __name:=rosbag_record')

        return Response({'mensaje': "Grabación iniciada con éxito"}, status=200)
    except Exception as e: 
        return Response({'mensaje' : f"Ocurrrió un error: {str(e)}"}, status=500)



@api_view(['POST'])
def stop_record(request):
    try:
        session_name = 'delete-bagfile-record-thread'
        session = server.find_where({"session_name": session_name})

        if not(session):
            session = server.new_session(session_name=session_name, kill_session=True, attach=False)

        window = session.attached_window
        pane = window.attached_pane
        pane.send_keys("export PATH=/usr/bin:/bin:/usr/sbin:/sbin")
        pane.send_keys("unset PYTHONPATH")
        # pane.send_keys("noetic")
        pane.send_keys("source ~/.bashrc")

        result = pane.send_keys("rosnode kill rosbag_record")
        print(result)


        # session_name = 'bagfile-record-thread'
        # session = server.find_where({"session_name": session_name})
        # if session:
            # session.kill_session()
        #
        return Response({'message' : 'Se detuvo la grabación'}, status = 200)

    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})



@api_view(['POST'])
def play_record(request):
    try:
        date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        play_bagfile(bagfile_name=os.path.join(FOLDER_RECORD_PATH, f'{date}.bag'), 
                     loop=False, 
                     rate=1.0, 
                     start_time=0.0, 
                     duration=30)

        return Response({'message' : 'Inicia playback de grabación'}, status = 200)
    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})



@api_view(['GET'])
def download_record(request):
    try:
        print('Descargando bagfile')
    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})



@api_view(['GET'])
def list_recorded_files(request):
    videos_folder = FOLDER_RECORD_PATH 
    try:
        files = [f for f in os.listdir(videos_folder) 
                 if os.path.isfile(os.path.join(videos_folder, f))]
        return JsonResponse({'files': files})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)



@api_view(['GET'])
def download_bag_file(request, file_name):

    file_path = os.path.join(FOLDER_RECORD_PATH, file_name)
    print(f'el archivo : {file_path}')
    if os.path.exists(file_path):
        print('el archivo existe') 
        return FileResponse(open(file_path, 'rb'), as_attachment=True, filename=file_name)
    else:
        return JsonResponse({'error': 'File not found'}, status=404)



@api_view(['DELETE'])
def delete_file(request, file_name):
    file_path = os.path.join(FOLDER_RECORD_PATH, file_name)
    print(file_path)
    if os.path.exists(file_path):
        os.remove(file_path)
        return JsonResponse({'message': 'File deleted successfully'})
    else:
        return JsonResponse({'error': 'File not found'}, status=404)



@api_view(['POST'])
def start_roslaunch(request):
    print('accessed')
    # result = ros_websocket_manager.start_roslaunch()
    session_name = 'websocket-server'
    command = 'roslaunch rosbridge_server rosbridge_websocket.launch'
    result = run_on_tmux_session(session_name, command, server)
    # print(result)
    # ros
    return JsonResponse({"message": result})


@api_view(['POST'])
def stop_roslaunch(request):
    # print('accessed')
    # result = ros_websocket_manager.stop_roslaunch()
    # print(result)
    session_name = 'websocket-server'
    stop_tmux_session(session_name, server)
    return JsonResponse({"message": 'result'})


@api_view(['POST'])
def convert_to_mp4(request, file_name):
    """Convert bag file to MP4 video using tmux command"""
    try:
        # Ensure converted_videos directory exists
        Path(FOLDER_MP4_PATH).mkdir(parents=True, exist_ok=True)

        # Input and output paths
        bag_file_path = os.path.join(FOLDER_RECORD_PATH, file_name)
        mp4_file_name = file_name.replace('.bag', '.mp4')
        mp4_file_path = os.path.join(FOLDER_MP4_PATH, mp4_file_name)

        # Check if bag file exists
        if not os.path.exists(bag_file_path):
            return JsonResponse({'error': 'Bag file not found'}, status=404)

        # Check if MP4 already exists
        if os.path.exists(mp4_file_path):
            return JsonResponse({'error': 'MP4 file already exists'}, status=409)

        # Build command to execute the conversion script with ROS environment
        script_path = os.path.join(settings.BASE_DIR, 'control', 'rosbag_utils', 'convert_bag_to_mp4.py')
        command = f'python3 {script_path} --bag-file {bag_file_path} --output-file {mp4_file_path}'

        # Use fixed session name for all conversions
        session_name = 'convert-bag-to-mp4-process'

        # Check if there's already a conversion running
        session = server.find_where({"session_name": session_name})
        if session:
            try:
                window = session.attached_window
                pane = window.attached_pane
                if pane.is_alive and pane.pid is not None:
                    # Another conversion is already running
                    return JsonResponse({
                        'error': 'Conversion already in progress',
                        'message': 'Another file is being converted. Please wait.',
                        'session_name': session_name,
                        'status': 'busy'
                    }, status=409)
                else:
                    # Session exists but process is dead - clean it up
                    print(f"Cleaning up dead conversion session '{session_name}'")
                    session.kill_session()
            except Exception as e:
                # Error checking session - clean it up anyway
                print(f"Error checking session '{session_name}': {e}")
                try:
                    session.kill_session()
                except:
                    pass

        # Start new conversion
        session_result = run_on_tmux_session(session_name, command, server)

        if session_result is True:
            return JsonResponse({
                'message': 'Conversion started in background',
                'session_name': session_name,
                'converting_file': file_name,
                'output_file': mp4_file_name,
                'status': 'started'
            })
        elif session_result is None:
            # This shouldn't happen since we check for active sessions above
            return JsonResponse({
                'error': 'Unexpected: Session already exists',
                'session_name': session_name,
                'status': 'already_running'
            }, status=409)
        else:
            return JsonResponse({
                'error': 'Failed to start conversion',
                'session_name': session_name,
                'status': 'error'
            }, status=500)

    except Exception as e:
        return JsonResponse({'error': f'Failed to start conversion: {str(e)}'}, status=500)


@api_view(['GET'])
def list_mp4_files(request):
    """List all converted MP4 files"""
    try:
        # Ensure directory exists
        Path(FOLDER_MP4_PATH).mkdir(parents=True, exist_ok=True)
        
        files = [f for f in os.listdir(FOLDER_MP4_PATH) 
                 if os.path.isfile(os.path.join(FOLDER_MP4_PATH, f)) and f.endswith('.mp4')]
        return JsonResponse({'files': files})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)


@api_view(['GET'])
def download_mp4_file(request, file_name):
    """Download MP4 file"""
    file_path = os.path.join(FOLDER_MP4_PATH, file_name)
    
    if os.path.exists(file_path):
        return FileResponse(open(file_path, 'rb'), as_attachment=True, filename=file_name)
    else:
        return JsonResponse({'error': 'File not found'}, status=404)


@api_view(['DELETE'])
def delete_mp4_file(request, file_name):
    """Delete MP4 file"""
    file_path = os.path.join(FOLDER_MP4_PATH, file_name)

    if os.path.exists(file_path):
        os.remove(file_path)
        return JsonResponse({'message': 'MP4 file deleted successfully'})
    else:
        return JsonResponse({'error': 'File not found'}, status=404)


@api_view(['GET'])
def conversion_status(request):
    """Check conversion status"""
    session_name = 'convert-bag-to-mp4-process'
    session = server.find_where({"session_name": session_name})

    if session:
        # Check if process is still active
        try:
            window = session.attached_window
            pane = window.attached_pane
            if pane.is_alive and pane.pid is not None:
                return JsonResponse({
                    'status': 'running',
                    'session_name': session_name,
                    'process_active': True,
                    'message': 'Conversion is currently running'
                })
            else:
                return JsonResponse({
                    'status': 'session_dead',
                    'session_name': session_name,
                    'process_active': False,
                    'message': 'Session exists but process has finished'
                })
        except Exception as e:
            return JsonResponse({
                'status': 'error_checking',
                'session_name': session_name,
                'error': str(e)
            })
    else:
        return JsonResponse({
            'status': 'idle',
            'session_name': session_name,
            'message': 'No conversion session active'
        })


@api_view(['POST'])
def stop_conversion(request):
    """Stop conversion process"""
    session_name = 'convert-bag-to-mp4-process'
    session = server.find_where({"session_name": session_name})

    if session:
        # Check if process is still active before stopping
        try:
            window = session.attached_window
            pane = window.attached_pane
            if pane.is_alive and pane.pid is not None:
                stop_tmux_session(session_name, server)
                return JsonResponse({
                    'message': 'Conversion stopped successfully',
                    'session_name': session_name,
                    'was_running': True
                })
            else:
                # Process already finished, just clean up session
                stop_tmux_session(session_name, server)
                return JsonResponse({
                    'message': 'Session cleaned up (process was already finished)',
                    'session_name': session_name,
                    'was_running': False
                })
        except Exception as e:
            stop_tmux_session(session_name, server)
            return JsonResponse({
                'message': 'Session stopped (with errors during check)',
                'session_name': session_name,
                'error': str(e)
            })
    else:
        return JsonResponse({
            'message': 'No active conversion session found',
            'session_name': session_name
        })

