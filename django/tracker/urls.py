from django.conf.urls.defaults import *
from tracker import settings
from tracker.track import views

# Uncomment the next two lines to enable the admin:
from django.contrib import admin
admin.autodiscover()

urlpatterns = patterns('',
    # Example:
    # (r'^tracker/', include('tracker.foo.urls')),
    (r'^$', views.view_project),
    (r'^view/entity/(?P<uuid>[a-f0-9\-]+)/$', views.view_entity),
    (r'^view/bag/(?P<plural_name>[a-zA-Z0-9\-]+)/$', views.view_bag),
    (r'^edit/entity/(?P<uuid>[a-f0-9\-]+)/$', views.edit_entity),
    (r'^add/entity/(?P<plural_name>[a-zA-Z0-9\-]+)/$', views.add_entity),

    # Uncomment the admin/doc line below and add 'django.contrib.admindocs' 
    # to INSTALLED_APPS to enable admin documentation:
    # (r'^admin/doc/', include('django.contrib.admindocs.urls')),
    
    #login/logout/some pitz account admin screens
    (r'^accounts/login/$', 'django.contrib.auth.views.login', {'template_name': 'login.html'}),
    (r'^accounts/logout/$', 'django.contrib.auth.views.logout', {'template_name': 'logout.html'}),
    (r'^accounts/user/(?P<uuid>[a-f0-9\-]+)/$', views.useruuids),

    # Uncomment the next line to enable the admin:
    (r'^admin/', include(admin.site.urls)),
)
#to enable django serve static files (e.g. css/javascript) the django static server has to be added to the urls
#else it should be served by a normal webserver (eg apache)
if settings.DEBUG:
    urlpatterns += patterns('',
        (r'^static/(?P<path>.*)$', 'django.views.static.serve', {'document_root': settings.MEDIA_ROOT}),
    )
