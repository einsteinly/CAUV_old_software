from django.shortcuts import render_to_response, redirect#responses
from django.http import Http404 #404 error
from django.template import RequestContext #used by django to prevent cross site request forgery for forms (django will through an error if form submitted without csrf token)
from django.contrib.auth.decorators import permission_required, login_required #Use @login_required to require login for a view
from django.forms.formsets import formset_factory
from pitz.project import Project
from pitz.bag import Bag
from pitz.entity import Entity
from uuid import UUID
from tracker import settings
from tracker.track import models, appforms, extras
import json

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
	if self.name == 'pscore' and value is not None and value != u"None":
	    self.value = dict(appforms.pscore_choices)[int(value)]
	else:
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

#i dont think we need this anymore                
#class disp_bag_by_type():
#    def __init__(self, bag):
#        # split bag into dictionary by type
#        self.by_type = {}
#        for e in bag:
#            typeid = e['type']
#            if self.by_type.has_key(typeid):
#                self.by_type[typeid].append(e)
#            else:
#                self.by_type[typeid] = [e]

class disp_entity():
    def __init__(self, entity):
        self.properties = [disp_property(x, entity[x]) for x in entity]
        self.type = entity.plural_name
        self.title = entity['title']
        self.uuid = entity.uuid
        self.related = []
        #for each entity type
        for plural_name in entity.project.plural_names:
            #find all the fields of that entity
            all_fields = extras.get_all_variables(entity.project.plural_names[plural_name])
            #we only want fields that are allowed to contain this entity
            fields={}
            for k,v in all_fields.items():
                if isinstance(v, list):
                    if isinstance(entity, v[0]):
                        fields[k]=v
                elif isinstance(entity, v):
                    fields[k]=v
            #we want to append iff there were matching fields (not neccessarily mathcing entities)
            if len(fields):
                #define a filter
                rf = extras.filter_atleast_one(*[extras.filter_matches_dict({field_name:entity}) for field_name in fields])
                self.related.append((disp_bag(entity.project, entity.project.plural_names[plural_name], rf)))
    def is_person(self):
        return self.type=='people'

class disp_bag():
    def __init__(self, project, entity_class, bag_filter=extras.filter_null_obj()):
        self.entity_class = entity_class
        self.errors = {}
        self.bag_filter = bag_filter
        self.elements = filter(extras.ErrorCatcher(bag_filter, self.errors), entity_class.all())
    def title(self):
        return self.entity_class.plural_name
    def view_ref(self):
        return self.entity_class.plural_name+'/?'+self.bag_filter.to_string()
    def add_ref(self):
        return self.entity_class.plural_name+'/?'
    def js(self):
        try:
            filters=[]
            values=[]
            filters, values, counter = self.bag_filter.to_list(filters, values)
            js_string = 'filters='+json.dumps(filters)+';values='+json.dumps(values)+';all_filters='+json.dumps(extras.js_filters)+';filter_names='+json.dumps(extras.js_filters.keys())
            return js_string
        except Exception as inst:
            print inst

def view_project(request):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    bags = []
    try:
    	pitz_user = p.by_uuid(UUID(request.user.get_profile().uuid))
    except:
    	pass
    for plural_name in p.plural_names:
        try:
            bags.append(disp_bag(p,p.plural_names[plural_name]))
        except AttributeError:
            continue
    return render_to_response('view_project.html', locals(), context_instance=RequestContext(request))
    
def view_bag(request, plural_name):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    entity_class = p.plural_names[plural_name]
    #order_form = appforms.make_order_form(p.plural_names[plural_name])
    if request.GET:
        bag = disp_bag(p, entity_class, extras.filter_from_GET(request.GET))
    #    order_forms = formset_factory(order_form, extra=2)(request.GET, prefix='order')
    #    filter_form = appforms.make_filter_form(p.plural_names[plural_name], request.GET)
    #    pitz_bag = filter_form.filter_bag(pitz_bag)
    #    order_list = []
    #    for form in order_forms.forms:
    #        if form.is_valid() and len(form.cleaned_data):
    #            order_list.append((form.cleaned_data['order_by'],form.cleaned_data['reverse']))
    #    def special_cmp(e1,e2):
    #        for o in order_list:
    #            result = cmp(e1[str(o[0])],e2[str(o[0])])
    #            if result != 0:
    #               if o[1]:
    #                   result=-result
    #               return result
    #        return 0
    #    pitz_bag.order(order_method=special_cmp)
    #else:
    #    order_forms = formset_factory(order_form, extra=2)(prefix='order')
    #    filter_form = appforms.make_filter_form(p.plural_names[plural_name])
    else:
        bag = disp_bag(p, entity_class)#filter_form.get_filter_link())
    return render_to_response('view_bag.html', locals(), context_instance=RequestContext(request))
    
def view_entity(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    e = get_entity_or_404(uuid, p)
    entity = disp_entity(e)
    return render_to_response('view_entity.html', locals(), context_instance=RequestContext(request))
    
@login_required
def edit_entity(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    try:
        user = p.by_uuid(UUID(request.user.get_profile().uuid))
    except models.UserProfile.DoesNotExist:
        user = p.people.matches_dict(**{'title': 'no owner'})[0]
    entity = get_entity_or_404(uuid, p)
    entity_form = appforms.make_entity_form(p.classes[entity['type']], p, entity)
    if request.method == 'POST':
        form = entity_form(request.POST)
        if form.is_valid():
            form.save(user)
            return redirect(to=settings.ROOT_URL+'/view/entity/'+str(uuid)+'/')
    else:
        form = entity_form()
    entity = disp_entity(entity)
    return render_to_response('form.html', locals(), context_instance=RequestContext(request))

@login_required
def add_entity(request, plural_name):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    try:
        user = p.by_uuid(UUID(request.user.get_profile().uuid))
    except models.UserProfile.DoesNotExist:
        user = p.people.matches_dict(**{'title': 'no owner'})[0]
    entity_form = appforms.make_entity_form(p.plural_names[plural_name], p)
    if request.method == 'POST':
        form = entity_form(request.POST)
        if form.is_valid():
            form.save(user)
            return redirect(to=settings.ROOT_URL+'/view/entity/'+str(form.entity.uuid)+'/')
    elif request.method == 'GET':
        i={}
        for x in request.GET:
            if isinstance(entity_form.base_fields[x], list):
                i[x] = [request.GET[x],]
            else:
                i[x] = request.GET[x]
        form = entity_form(i)
    else:
        form = entity_form()
    entity = disp_entity(form.entity)
    return render_to_response('form.html', locals(), context_instance=RequestContext(request))

@permission_required('is_staff')
def useruuids(request, uuid):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    user = get_entity_or_404(uuid, p)
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
    entity = disp_entity(user)
    return render_to_response('form.html', locals(), context_instance=RequestContext(request))

