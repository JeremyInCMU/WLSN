"""WLSTermProject URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/1.9/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  url(r'^$', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  url(r'^$', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.conf.urls import url, include
    2. Add a URL to urlpatterns:  url(r'^blog/', include('blog.urls'))
"""
from django.conf.urls import url
from django.contrib import admin
from django.views.generic import TemplateView
from iotEasy.views import loadDataRequest
from iotEasy.views import addDataRequest
from iotEasy.views import sendEmail


urlpatterns = [
    url(r'^admin/', admin.site.urls),
    url(r'^$', TemplateView.as_view(template_name="iotEasy.html")),
    url(r'^node_1/',TemplateView.as_view(template_name="node_1.html")),
    url(r'^node_2/',TemplateView.as_view(template_name="node_2.html")),
    url(r'^node_3/',TemplateView.as_view(template_name="node_3.html")),
    url(r'^node_4/',TemplateView.as_view(template_name="node_4.html")),
    url(r'^node_5/',TemplateView.as_view(template_name="node_5.html")),
    url(r'^node_6/', TemplateView.as_view(template_name="node_6.html")),
    url(r'^node_7/', TemplateView.as_view(template_name="node_7.html")),
    url(r'^node_8/', TemplateView.as_view(template_name="node_8.html")),
    url(r'^loadData/', loadDataRequest, name="loadDataURL"),
    url(r'^addData/',addDataRequest,name="addDataURL"),
    url(r'^sendEmail/',sendEmail,name="sendEmailURL"),
    url(r'^dataAcquisition/', loadDataRequest, name='dataAcquisitionURL'),
    url(r'^initTimeData/', loadDataRequest, name='initTimeDataURL'),
]

