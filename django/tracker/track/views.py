from django.shortcuts import render_to_response#responses
from django.http import Http404 #404 error
from django.template import RequestContext #used by django to prevent cross site request forgery for forms (django will through an error if form submitted without csrf token)
from django.contrib.auth.decorators import login_required #Use @login_required to require login for a view
from pitz.project import Project
from pitz.bag import Bag
from pitz.entity import Entity
from uuid import UUID
from tracker import settings

def view(request, uuid=''):
    p = Project.from_pitzdir(settings.PITZ_DIR)
    items = []
    if uuid == '': #if theres no uuid provided, display the whole project
        for type in p.plural_names:
            try:
                try:
                    i = p.__getattribute__(type)
                    items.append(((i.title,i.uuid),[(x.title, x.uuid) for x in i])) # do bags have uuids? instanceofbag.uuid gives a uuid, but it appears to be randomly generated and doesnt return the bag when used to look it up in the project
                except AttributeError:
                    continue
            except ValueError:
                items[type] = 'Broken'
    else: # uuid provided
        object = p.by_uuid(UUID(uuid))
        if isinstance(object, Bag): #see above note, bags dont seem to have valid uuids
            items = [((object.title),[(object.contents)])]
        elif isinstance(object, Entity):
            items = [[[x],[[object[x]]]] for x in object]
        else:#couldnt find the object, so raise 404
            raise Http404
    return render_to_response('view.html', locals())
