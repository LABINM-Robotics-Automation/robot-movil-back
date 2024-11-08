from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from . rosbag_utils.main import * 
from django.conf import settings
import os

PATH_FILE = os.path.join(settings.BASE_DIR, 'control', 'rosbag_utils', 'start-zed2i-camera.sh') 

@api_view(['POST'])
def start_camera(request):
    try:
        camera_process = execute_bash(PATH_FILE, wait=False)
        return Response({'mensaje': f"Zed2i iniciada en proceso {camera_process}"}, status=200)

    except Exception as e: 
        return Response({ 'mensaje' : f"Ocurrrió un error: {str(e)}" }, status=500)


@api_view(['POST'])
def stop_camera(request):
    try:
        execute_bash('rosnode kill /zed2i/zed_node')
        return Response({'mensaje': f"Zed2i stopped"},status=200)

    except Exception as e: 
        return Response({'mensaje' : f"Ocurrrió un error: {str(e)}"}, status=500)


@api_view(['POST'])
def start_record(request):
    try:
        date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        process = record_bagfile(topics=['/zed2i/zed_node/rgb_raw/image_raw_color'], 
                                 name=f'{date}.bag',
                                 basedir = os.path.join(settings.BASE_DIR,'control','videos'))

        return Response({'mensaje': f"Zed2i iniciada en proceso {process}"}, status=200)
    except Exception as e: 
        return Response({'mensaje' : f"Ocurrrió un error: {str(e)}"}, status=500)


@api_view(['POST'])
def stop_record(request):
    try:
        execute_bash('rosnode kill rosbag_record')
        return Response({'message' : f'Se detuvo la grabación'}, status = 200)

    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})


from datetime import datetime

FOLDER_RECORD_PATH = os.path.join(settings.BASE_DIR, 'control', 'videos')

@api_view(['POST'])
def play_record(request):
    try:
        date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        play_bagfile(bagfile_name=os.path.join(FOLDER_RECORD_PATH, f'{date}.bag'), 
                     loop=False, 
                     rate=1.0, 
                     start_time=0.0, 
                     duration=30)

        return Response({'message' : f'Inicia playback de grabación'}, status = 200)
    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})


@api_view(['GET'])
def download_record(request):
    try:
        print('Descargando bagfile')
    except Exception as e:
        return Response({'message' : f'Ocurrio un error : {str(e)}'})

from django.http import JsonResponse
import os

@api_view(['GET'])
def list_recorded_files(request):
    videos_folder = FOLDER_RECORD_PATH  # Replace with the actual path to the 'videos' folder
    
    try:
        # Get a list of all files in the folder
        files = [f for f in os.listdir(videos_folder) 
                 if os.path.isfile(os.path.join(videos_folder, f))]
        return JsonResponse({'files': files})
    except Exception as e:
        return JsonResponse({'error': str(e)}, status=500)

