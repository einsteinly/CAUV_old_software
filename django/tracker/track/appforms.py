from django import forms
from tracker.track import models, extras
from datetime import datetime
from pitz.entity import Entity, Activity
from uuid import UUID
from tracker import settings

class useruuid_form(forms.ModelForm):
    class Meta:
        model = models.UserProfile
        exclude = ('uuid')

#textarea and pscore  fields (since we cant pass the required args to charfield when we call it)
class Textarea(forms.CharField):
    def __init__(self, **kwargs):
        super(Textarea, self).__init__(widget=forms.Textarea, **kwargs)

pscore_choices = ((0, 'Useless'),
                  (1, 'Would be nice'),
		  (2, 'Long term goal'),
		  (3, 'Short term goal'),
		  (4, 'Urgent'),
		  (5, 'Critical'))

class PscoreField(forms.ChoiceField):
    def __init__(self, **kwargs):
        super(PscoreField, self).__init__(choices = pscore_choices, **kwargs)

#defs for first run fields
type_to_field = {
                datetime: forms.DateField,
                int: forms.IntegerField,
                str: Textarea,
		unicode: Textarea
                    }

name_to_field = {
                'title': forms.CharField,
		'pscore': PscoreField
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
    variables = extras.get_all_variables(entity_type)
    fields={}
    for var_name in variables:
        if isinstance(variables[var_name], list):
            if issubclass(variables[var_name][0], Entity):
                fields[var_name] = get_EntityMultipleChoiceField(variables[var_name][0])
        elif issubclass(variables[var_name], Entity):
            fields[var_name] = get_EntityChoiceField(variables[var_name])
        else:
            try:
	        fields[var_name] = name_to_field[var_name]
	    except KeyError:
	        try:
		    print variables[var_name]
                    fields[var_name] = type_to_field[variables[var_name]]
                except KeyError: #fallback on text field
                    fields[var_name] = forms.CharField
    return fields
        
def make_entity_form(entity_type, project, initial=None):
    field_classes = analyse(entity_type) # get field types +names
    fields = {}
    
    if initial: #if were given some data, try and generate from said data, otherwise fall back to default, then empty
        entity = initial
        for field_class in field_classes:
            try:
                fields[field_class] = field_classes[field_class](initial=initial[field_class], required=False)
            except KeyError:
                fields[field_class] = field_classes[field_class](required=False)
        isnew = False
    else:
        for field_class in field_classes:
            fields[field_class] = field_classes[field_class](required=False)
        i = {}
        for x in entity_type.required_fields:
            if entity_type.required_fields[x] == None:
                i[x] = ''
            elif callable(entity_type.required_fields[x]):
                i[x] = entity_type.required_fields[x](project)
            else:
                i[x] = entity_type.required_fields[x]
        entity = entity_type(**i)#need to set required fields
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
            self.entity['created_by'] = user
        if len(changed):
            activity = Activity(title=str(user)+" changed "+str(self.entity),description='\n'.join([str(x[0])+" changed to "+str(x[1]) for x in changed]), created_by=user, who_did_it=user, entity=self.entity)
            activity.to_yaml_file(settings.PITZ_DIR)
            self.entity.to_yaml_file(settings.PITZ_DIR)
        return
    return type('entity_form', (forms.BaseForm,), { 'base_fields': fields, 'entity':entity, 'isnew': isnew, 'save': save })

def make_order_filter_forms(entity_type):
    analysis = analyse(entity_type)
    choices = [(x, x) for x in analysis]
    choices.append(('','None'))
    class order_form(forms.Form):
        order_by = forms.ChoiceField(choices=choices,label='Order by',initial='')
        reverse = forms.BooleanField(label='Reverse', required=False)
    class filter_form(forms.Form):
        a = forms.ChoiceField(choices=choices, label='Filter by')
        b = forms.ChoiceField(choices=(('lte','<='),('gte','>='),('eq','==')),label='',initial='eq')
        c = forms.CharField(label='')
    return order_form, filter_form
