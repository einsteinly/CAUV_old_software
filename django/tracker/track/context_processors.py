def root_url(request):
    from django.conf import settings
    return {'ROOT_URL': settings.ROOT_URL}
