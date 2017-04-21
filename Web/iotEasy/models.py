from django.db import models

# Create your models here.
class Data(models.Model):
    NodeID = models.CharField(max_length=10)
    Time = models.CharField(max_length=10)
    Accelerometer = models.CharField(max_length=10)
    Pressure = models.CharField(max_length=10)
    Light = models.CharField(max_length=10)
    Sound = models.CharField(max_length=10)
    Motion = models.CharField(max_length=10)
    Humidity = models.CharField(max_length=10)
    Thermometer = models.CharField(max_length=10)

