from django.shortcuts import render_to_response, redirect#responses
from django.http import Http404 #404 error
from django.template import RequestContext #used by django to prevent cross site request forgery for forms (django will through an error if form submitted without csrf token)
from django.contrib.auth.decorators import permission_required, login_required #Use @login_required to require login for a view
from pitz.project import Project
from pitz.bag import Bag
from pitz.entity import Entity
from uuid import UUID
from tracker import settings
from tracker.track import models, appforms

def get_entity_or_404(uuid, project=None):
    uuid_obj = UUID(uuid)
    if not project:
        project = Project.from_pitzdir(settings.PITZ_DIR)
    try:
        e = project.by_uuid(uuid_obj)
    except ValueError:
        raise Http404
    if not isinstance(e, Entity):
        raise Http404
    return e
    
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

class disp_bag_by_type():
    def __init__(self, bag):
        # split bag into dictionary by type
        self.by_type = {}
        for e in bag:
            typeid = e['type']
            if self.by_type.has_key(typeid):
                self.by_type[typeid].append(e)
            else:
                self.by_type[typeid] = [e]

class disp_entity():
    def __init__(self, entity, related=[]):
        self.properties = [disp_property(x, entity[x]) for x in entity]
        self.type = entity.plural_name
        self.title = entity['title']
        self.uuid = entity.uuid
        # fingers crossed...
        #for x in related:
        #    print disp_entity(x)
        #    print x['title']
        #    print x.plural_name
        #    print [disp_property(y, x[y]) for y in x]
        #self.related = [disp_entity(x) for x in related]
        self.related = disp_bag_by_type(related)
    def is_person(self):
        return self.type=='people'

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
    e = get_entity_or_404(uuid, p)
    # # find things which have this entity listed as their entity... like
    # # comments :)
    # related = p.matches_dict(entity=e)

    # find anything with any field as this entity (eep!)
    related = []
    for k in [x[0] for x in p.attributes]:
        # python magic: pass dictionary as explicit kwargs:
        matches = p.matches_dict(**{k:e})
        # TODO: faster intersection...
        for m in matches:
            if not m in related:
                related.append(m)
    entity = disp_entity(e, related)
    return render_to_response('view_entity.html', locals())
    
@login_required
def edit_entity(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    entity = get_entity_or_404(uuid, p)
    entity_form = appforms.make_entity_form(entity)
    if request.method == 'POST':
        form = entity_form(request.POST)
        if form.is_valid():
            form.save()
            return redirect(to='/view/entity/'+uuid+'/')
    else:
        form = entity_form()
    return render_to_response('form.html', locals())

@permission_required('is_staff')
def useruuids(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    e = get_entity_or_404(uuid, p)
    if not isinstance(user, p.classes['person']):
        raise Http404 #if someone tries to set some non-person entity as that persons user
    if request.method == 'POST':
        form = appforms.useruuid_form(request.POST)
        if form.is_valid():
            profile = form.save(commit=False)
            profile.uuid = uuid
            profile.save()
            return redirect(to='/view/entity/'+uuid+'/')
    else:
        try:
            form = appforms.useruuid_form(instance=models.UserProfile.objects.get(uuid=uuid))
        except models.UserProfile.DoesNotExist:
            form = appforms.useruuid_form()
    return render_to_response('form.html', locals())
    