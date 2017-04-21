from django.shortcuts import render

# Create your views here.

# from django.shortcuts import render
from django.http import HttpResponse
from iotEasy.models import Data
from django.core.mail import EmailMessage
# Create your views here.

def loadDataRequest(request):
    dataSet = Data.objects.using('users').all()
    result = ""
    for data in dataSet:
        result = result + data.NodeID+","
        result = result + data.Time+","
        result = result + data.Accelerometer+","
        result = result + data.Pressure+","
        result = result + data.Light+","
        result = result + data.Sound+","
        result = result + data.Motion+","
        result = result + data.Humidity+","
        result = result + data.Thermometer+";"

    return HttpResponse(result)


def addDataRequest(request):
    try:
        NodeID = request.GET["NodeID"]
        Time = request.GET["Time"]
        Accelerometer = request.GET["Accelerometer"]
        Pressure = request.GET["Pressure"]
        Light = request.GET["Light"]
        Sound = request.GET["Sound"]
        Motion = request.GET["Motion"]
        Humidity = request.GET["Humidity"]
        Thermometer = request.GET["Thermometer"]
        newData = Data(NodeID = NodeID,Time=Time,Accelerometer=Accelerometer,
                       Pressure=Pressure,Light=Light,Sound=Sound,Motion=Motion,
                       Humidity=Humidity,Thermometer=Thermometer)
        newData.save(using='users')
        return HttpResponse("Success")
    except:
        return HttpResponse("Fail")


def sendEmail(request):
    command = request.GET.get('Command')
    try:
        if command != '':
            email = EmailMessage('Sensor Alert', command,
                                 to=['jiamingx90@gmail.com'])
            email.send()
            return HttpResponse("Send")
        else:
            return HttpResponse("Not Send")
    except:
        return HttpResponse("Fail")
