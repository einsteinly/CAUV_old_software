from django import forms
from tracker.track import models, extras
from datetime import datetime
from pitz.entity import Entity, Activity
from uuid import UUID
from tracker import settings

#USER UUID FORM
class useruuid_form(forms.ModelForm):
    class Meta:
        model = models.UserProfile
        fields = ('uuid')
        
class preferences_form(forms.Form):
    class Meta:
        model = models.UserProfile
        exclude = ('uuid', 'user')
        
def make_pref_form(user_profile):
    class preferences_form(forms.ModelForm):
        class Meta:
            model = models.UserProfile
            exclude = ('uuid', 'user')
        shortcuts = forms.ModelMultipleChoiceField(queryset=user_profile.usershortcut_set.all())
        def __init__(self, *args, **kwargs):
            if 'instance' in kwargs:
                initial = kwargs.setdefault('initial', {})
                initial['shortcuts'] = [shortcut.pk for shortcut in kwargs['instance'].usershortcut_set.all()]
            forms.ModelForm.__init__(self, *args, **kwargs)
        def save(self):
            instance = forms.ModelForm.save(self)
            for shortcut in instance.usershortcut_set.all():
                shortcut.delete()
            for shortcuts in self.cleaned_data['shortcuts']:
                instance.usershortcut_set.add(shortcut)
    return preferences_form

#ENTITY FORM
#any pitz fields that require something special (like a widget) that is specified when the class is called

#TEXTAREA FIELD: used anywhere were more space is needed (e.g. description field)
class Textarea(forms.CharField):
    def __init__(self, **kwargs):
        super(Textarea, self).__init__(widget=forms.Textarea, **kwargs)

#PSCORE FIELD: since Pscore is a number, this maps it to a more useful description
#this defines the descriptions that map to pscores. Note that changing these will mess up any already set
pscore_choices = ((None, 'N/A'),
                  (0, 'Useless'),
                  (1, 'Would be nice'),
                  (2, 'Long term goal'),
                  (3, 'Short term goal'),
                  (4, 'Urgent'),
                  (5, 'Critical'))

class PscoreField(forms.ChoiceField):
    def __init__(self, **kwargs):
        super(PscoreField, self).__init__(choices = pscore_choices, **kwargs)

#DEFINES DEFAULT BEHAVIOUR FOR FIELDS
type_to_field = {
                datetime: forms.DateField,
                bool: forms.BooleanField,
                int: forms.IntegerField,
                str: Textarea,
                unicode: Textarea
                    }

name_to_field = {
                'title': forms.CharField,
                'pscore': PscoreField,
                }

#ENTITY FIELDS
def get_EntityChoiceField(entity_type, project):
    """
    Given an entity type and a project, returns the class of a django single choice field with the right choices and clean method
    Note that although entity type has an all method, this doesn't work in all cases
    """
    class EntityChoiceField(forms.ChoiceField):
        def __init__(self, initial=None, **kwargs):
            if initial:
                super(EntityChoiceField, self).__init__(choices=[[e.uuid, e] for e in project if isinstance(e, entity_type)], initial=initial.uuid, **kwargs)
            else:
                super(EntityChoiceField, self).__init__(choices=[[e.uuid, e] for e in project if isinstance(e, entity_type)], **kwargs)
        def clean(self, value):
            uuid = super(EntityChoiceField, self).clean(value)
            if uuid:
                return project.by_uuid(UUID(uuid))
            return None
    #these fields are needed elsewhere to chec
    EntityChoiceField.entity_type = entity_type
    EntityChoiceField.is_entitychoice = 1
    return EntityChoiceField

def get_EntityMultipleChoiceField(entity_type, project): 
    """
    Given an entity type and a project, returns the class of a django multiple choice field with the right choices and clean method
    Note that although entity type has an all method, this doesn't work in all cases
    """       
    class EntityMultipleChoiceField(forms.MultipleChoiceField):
        def __init__(self, initial=None, **kwargs):
            if initial:
                super(EntityMultipleChoiceField, self).__init__(choices=[[e.uuid, e] for e in project if isinstance(e, entity_type)], initial=[e.uuid for e in initial], **kwargs)
            else:
                super(EntityMultipleChoiceField, self).__init__(choices=[[e.uuid, e] for e in project if isinstance(e, entity_type)], **kwargs)
        def clean(self, value):
            uuids = super(EntityMultipleChoiceField, self).clean(value)
            return [project.by_uuid(UUID(uuid)) for uuid in uuids]
    EntityMultipleChoiceField.entity_type = entity_type
    EntityMultipleChoiceField.is_entitychoice = 2
    return EntityMultipleChoiceField

#ANALYSE FUNCTION
def analyse(entity_type, project):
    """
    Given an entity type and a project, generates a list of django field classes
    """
    variables = extras.get_all_variables(entity_type)
    fields={}
    for var_name in variables:
        if isinstance(variables[var_name], list):
            if issubclass(variables[var_name][0], Entity):
                fields[var_name] = get_EntityMultipleChoiceField(variables[var_name][0], project)
        elif issubclass(variables[var_name], Entity):
            fields[var_name] = get_EntityChoiceField(variables[var_name], project)
        else:
            try:
                fields[var_name] = name_to_field[var_name]
            except KeyError:
                try:
                    fields[var_name] = type_to_field[variables[var_name]]
                except KeyError: #fallback on text field
                    fields[var_name] = forms.CharField
    return fields

#ACTUAL FORM MAKING METHOD
def make_entity_form(entity_type, project, initial=None):
    """
    Given an entity_type and a project, generates a django form with approroate save method.
    If initial is specified, tries to fill the form with initial data (initial must be an entity)
    """
    #get the field dictionary
    field_classes = analyse(entity_type, project)
    fields = {}
    #if were given an entity, try and generate from entity, otherwise fall back to default, then empty
    if initial:
        #specifies the entity an entity associated with this form
        entity = initial
        #for each field try and generate it with the entity data, which throws a key error if the entity doesnt have it
        for field_class in field_classes:
            try:
                fields[field_class] = field_classes[field_class](initial=initial[field_class], required=False)
            except KeyError:
                fields[field_class] = field_classes[field_class](required=False)
        #specifies whether a new entity is being created
        isnew = False
    else:
        #generate the fields
        for field_class in field_classes:
            fields[field_class] = field_classes[field_class](required=False)
        i = {}
        #for each field check whether its a required field, then fill in data accordingly
        #pitz specifies default for some and not for others, these clauses work around that
        for x in entity_type.required_fields:
            if entity_type.required_fields[x] == None:
                i[x] = ''
            elif callable(entity_type.required_fields[x]):
                i[x] = entity_type.required_fields[x](project)
            else:
                i[x] = entity_type.required_fields[x]
        #create entity with these default values
        entity = entity_type(**i)
        isnew = True
    #specify the project the entity is associated with
    entity.project=project
    def save(self, user):
        """
        Save method for form. Takes the data, checks whether its changed, tries to save it and generate an activity entity
        """
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
            activity.project = project
            activity.to_yaml_file(settings.PITZ_DIR)
            self.entity.to_yaml_file(settings.PITZ_DIR)
        return
    return type('entity_form', (forms.BaseForm,), { 'base_fields': fields, 'entity':entity, 'isnew': isnew, 'save': save})