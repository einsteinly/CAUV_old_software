from django.shortcuts import render_to_response, redirect, get_object_or_404 #responses
from django.utils.safestring import mark_safe
from django.template import RequestContext #used by django to prevent cross site request forgery for forms (django will through an error if form submitted without csrf token)
from django.contrib.auth.decorators import login_required #Use @login_required to require login for a view
from pitz.project import Project
from pitz.bag import Bag
from pitz.entity import Entity
from tracker import settings

def view(request, uid=''):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    items = {}
    for type in p.plural_names:
        try:
            try:
                items[type] = str(p.__getattribute__(type))
            except AttributeError:
                items[type] = 'None' #converts dict to list, if you can get the template to use a dict then that would be better
        except ValueError:
            items[type] = 'Broken'
    return render_to_response('view.html', locals())
