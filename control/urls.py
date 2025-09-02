from django.urls import path
from . import views
# from .classes.zedImageView import ZedImageView

urlpatterns = [
    path('start_camera', views.start_camera, name='start_camera'),
    path('stop_camera', views.stop_camera, name='stop_camera'),
    path('start_record', views.start_record, name='start_record'),
    path('stop_record', views.stop_record, name='stop_record'),
    path('play_record', views.play_record, name='play_record'),
    path('list_files/', views.list_recorded_files, name='list_recorded_files'),
    path('download/<str:file_name>', views.download_bag_file, name='download_bag_file'),
    path('delete/<str:file_name>', views.delete_file, name='delete_file'),
    path("start_websocket", views.start_roslaunch, name="start_roslaunch"),
    path("stop_websocket", views.stop_roslaunch, name="stop_roslaunch"),
    path("start_image_processor", views.start_image_processor, name="start_image_processor"),
    path("stop_image_processor", views.stop_image_processor, name="stop_image_processor"),
    
    # MP4 conversion endpoints
    path('convert_to_mp4/<str:file_name>', views.convert_to_mp4, name='convert_to_mp4'),
    path('conversion_status/', views.conversion_status, name='conversion_status'),
    path('stop_conversion/', views.stop_conversion, name='stop_conversion'),
    path('list_mp4_files/', views.list_mp4_files, name='list_mp4_files'),
    path('download_mp4/<str:file_name>', views.download_mp4_file, name='download_mp4_file'),
    path('delete_mp4/<str:file_name>', views.delete_mp4_file, name='delete_mp4_file'),
]

