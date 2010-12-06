from django import forms
from tracker.track import models
from datetime import datetime
from pitz.entity import Entity, Activity
from uuid import UUID
from tracker import settings

class useruuid_form(forms.ModelForm):
    class Meta:
        model = models.UserProfile
        exclude = ('uuid')

#textarea fields (since we cant pass the required args to charfield when we call it)
class Textarea(forms.CharField):
    def __init__(self, **kwargs):
        super(Textarea, self).__init__(widget=forms.Textarea, **kwargs)

#defs for first run fields
basic_fields = {'description': Textarea, 'title': forms.CharField}
type_to_field = {
                datetime: forms.DateField,
                int: forms.IntegerField
                    }

def get_EntityChoiceField(entity_type):                    
    class EntityChoiceField(forms.ChoiceField):
        def __init__(self, initial=None, **kwargs):
            if initial:
                super(EntityChoiceField, self).__init__(choices=[[e.uuid, e] for e in self.entity_type.all()], initial=initial.uuid, **kwargs)
            else:
                super(EntityChoiceField, self).__init__(choices=[[e.uuid, e] for e in self.entity_type.all()], **kwargs)
        def clean(self, value):
            uuid = super(EntityChoiceField, self).clean(value)
            if uuid:
                return self.entity_type.all().by_uuid(UUID(uuid))
            return None
    EntityChoiceField.entity_type = entity_type
    return EntityChoiceField

def get_EntityMultipleChoiceField(entity_type):        
    class EntityMultipleChoiceField(forms.MultipleChoiceField):
        def __init__(self, initial=None, **kwargs):
            if initial:
                super(EntityMultipleChoiceField, self).__init__(choices=[[e.uuid, e] for e in self.entity_type.all()], initial=[e.uuid for e in initial], **kwargs)
            else:
                super(EntityMultipleChoiceField, self).__init__(choices=[[e.uuid, e] for e in self.entity_type.all()], **kwargs)
        def clean(self, value):
            uuids = super(EntityMultipleChoiceField, self).clean(value)
            entities = self.entity_type.all()
            return [entities.by_uuid(UUID(uuid)) for uuid in uuids]
    EntityMultipleChoiceField.entity_type = entity_type
    return EntityMultipleChoiceField
        
def analyse(entity_type):
    fields = basic_fields
    entity_type.allowed_types.update(Entity.allowed_types)
    for data_type in entity_type.allowed_types:
        if isinstance(entity_type.allowed_types[data_type], list):
            if issubclass(entity_type.allowed_types[data_type][0], Entity):
                fields[data_type] = get_EntityMultipleChoiceField(entity_type.allowed_types[data_type][0])
        elif issubclass(entity_type.allowed_types[data_type], Entity):
            fields[data_type] = get_EntityChoiceField(entity_type.allowed_types[data_type])
        else:
            try:
                fields[data_type] = type_to_field[type(entity_type.allowed_types[data_type])]
            except KeyError: #fallback on text field
                fields[data_type] = forms.CharField
    return fields
        
def make_entity_form(entity_type, initial=None):
    field_classes = analyse(entity_type)
    fields = {}
    if initial:
        entity = initial
        for field_class in field_classes:
            try:
                print field_class
                fields[field_class] = field_classes[field_class](initial=initial[field_class], required=False)
            except KeyError:
                fields[field_class] = field_classes[field_class](required=False)
        isnew = False
    else:
        for field_class in field_classes:
            fields[field_class] = field_classes[field_class](required=False)
        entity = entity_type(title='',description='')#need to set description + title otherwise we get errors
        isnew = True
    def save(self, user):
        changed = []
        for x in self.cleaned_data:
            try:
                data_status = self.entity[x] == self.cleaned_data[x]
            except KeyError:
                data_status = False
            if not data_status: #attempts to reduce saving if no changes have been made (doesnt always work)
                changed.append((x, self.cleaned_data[x]))
                self.entity[x] = self.cleaned_data[x] #for each bit of data in the cleaned data, assign it to the entity
        if self.isnew:
            entity['created_by'] = user
        if len(changed):
            activity = Activity(title=str(user)+" changed "+str(entity),description='\n'.join([str(x[0])+" changed to "+str(x[1]) for x in changed]), created_by=user, who_did_it=user, entity=entity)
            activity.to_yaml_file(settings.PITZ_DIR)
            entity.to_yaml_file(settings.PITZ_DIR)
        return
    return type('entity_form', (forms.BaseForm,), { 'base_fields': fields, 'entity':entity, 'isnew': isnew, 'save': save })
