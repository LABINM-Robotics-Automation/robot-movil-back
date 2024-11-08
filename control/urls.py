from django.urls import path
from . import views
# from .classes.zedImageView import ZedImageView

urlpatterns = [
    path('start_camera', views.start_camera, name='start_camera'),
    path('stop_camera', views.stop_camera, name='stop_camera'),
    path('start_record', views.start_record, name='start_record'),
    path('stop_record', views.stop_record, name='stop_record'),
    path('play_record', views.play_record, name='play_record'),
    path('list_files/', views.list_recorded_files, name='list_recorded_files') 
    # path('zed_image', ZedImageView.as_view(), name='zed_image')
    # path('zed_image', views.zed_image, name='zed_image')
]

