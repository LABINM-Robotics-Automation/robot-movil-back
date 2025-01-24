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


FOLDER_RECORD_PATH = os.path.join(settings.BASE_DIR, 'control', 'videos')
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
        pane.send_keys("noetic")
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
        pane.send_keys("noetic")
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
        return HttpResponse("File not found", status=404)



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

