from django.shortcuts import render
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from . rosbag_utils.main import * 

PATH_FILE = '/home/pqbas/labinm/robot-movil-back/robot_movil_back/control/rosbag_utils/start-zed2i-camera.sh' 
     
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
        process = record_bagfile(topics=['/zed2i/zed_node/rgb_raw/image_raw_color'], 
                                 date='24-10-12',
                                 test_number='1', 
                                 description='detection-test')

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


RECORD_PATH = '/home/pqbas/labinm/robot-movil-back/robot_movil_back/detection-test_1_24-10-12.bag'
@api_view(['POST'])
def play_record(request):
    try:        
        play_bagfile(bagfile_name=RECORD_PATH, 
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
