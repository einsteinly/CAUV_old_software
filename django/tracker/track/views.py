from django.shortcuts import render_to_response#responses
from django.http import Http404 #404 error
from django.template import RequestContext #used by django to prevent cross site request forgery for forms (django will through an error if form submitted without csrf token)
from django.contrib.auth.decorators import login_required #Use @login_required to require login for a view
from pitz.project import Project
from pitz.bag import Bag
from pitz.entity import Entity
from uuid import UUID
from tracker import settings

#custom representations: pitz own repesentation methods are orientated to the command line
class disp_property():
    def __init__(self, name, value):
        self.name = name
        self.value = value
        if isinstance(value, list):
            try:   
                if isinstance(value[0], Entity):
                    self.type = 'entitylist'
                else:
                    self.type = 'valuelist'
            except IndexError:
                self.type='valuelist'
        else:
            if isinstance(value, Entity):
                self.type = 'entity'
            else:
                self.type = 'value'

class disp_entity():
    def __init__(self, entity):
        self.properties = [disp_property(x, entity[x]) for x in entity]
        self.type = entity.plural_name
        self.title = entity['title']
        self.subs = []
       
class disp_bag():
    def __init__(self, bag, ref):
        self.title = bag.title
        self.ref = ref
        self.elements = bag._elements

def view_project(request):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    bags = []
    for type in p.plural_names:
        try:
            bags.append(disp_bag(p.__getattribute__(type),type))
        except AttributeError:
            continue
    return render_to_response('view_project.html', locals())
    
def view_bag(request, ref):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    bag = disp_bag(p.__getattribute__(ref),ref)
    return render_to_response('view_bag.html', locals())
    
def view_entity(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    try:
        e = p.by_uuid(UUID(uuid))
    except ValueError:
        raise Http404
    if not isinstance(e, Entity):
        raise Http404
    entity = disp_entity(e)
    return render_to_response('view_entity.html', locals())
