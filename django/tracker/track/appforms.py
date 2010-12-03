from django import forms
from tracker.track import models
from datetime import datetime
from pitz.entity import Entity
from uuid import UUID
from tracker import settings

class useruuid_form(forms.ModelForm):
    class Meta:
        model = models.UserProfile
        exclude = ('uuid')

known_pitz_fields = ('created_time', 'yaml_file_saved', 'uuid', 'type', 'frag')
known_auto_fields = {'modified_time': datetime.now}
type_to_field = {
                datetime: forms.DateField,
                int: forms.IntegerField
                    }
                    
class EntityChoiceField(forms.ChoiceField):
    def __init__(self, entity):
        super(EntityChoiceField, self).__init__(choices=[[e.uuid, e] for e in entity.all()], initial=entity.uuid, required=False)
        self.entity = entity
    def clean(self, value):
        uuid = super(EntityChoiceField, self).clean(value)
        return self.entity.all().by_uuid(UUID(uuid))
        
class EntityMultipleChoiceField(forms.MultipleChoiceField):
    def __init__(self, entity):
        super(EntityMultipleChoiceField, self).__init__(choices=[[e.uuid, e] for e in entity[0].all()], initial=[e.uuid for e in entity], required=False)
        self.entity = entity
    def clean(self, value):
        uuids = super(EntityMultipleChoiceField, self).clean(value)
        entities = self.entity[0].all()
        return [entities.by_uuid(UUID(uuid)) for uuid in uuids]
        
def analyse(entity):
    fields = {}
    auto_fields = {}
    for x in entity:
        if not x in known_pitz_fields: #if pitz doesnt generate it
            try:
                auto_fields[x] = known_auto_fields[x] #see if theres an autogeneration method
            except KeyError: #else try and find out what data type it is
                if isinstance(entity[x], list):
                    if len(entity[x]):
                        fields[x] = EntityMultipleChoiceField(entity[x])
                elif isinstance(entity[x], Entity):
                    fields[x] = EntityChoiceField(entity[x])
                else:
                    try:
                        fields[x] = type_to_field[type(entity[x])](initial=entity[x], required=False)
                    except KeyError: #fallback on text field
                        fields[x] = forms.CharField(widget=forms.Textarea(attrs={'rows':'','cols':''}),initial=entity[x], required=False)
    return fields, auto_fields
        
def make_entity_form(entity):
    fields, auto_fields = analyse(entity)
    def save(self):
        changed = False
        for x in self.cleaned_data:
            if not self.entity[x] == self.cleaned_data[x]: #attempts to reduce saving if no changes have been made (doesnt always work)
                self.entity[x] = self.cleaned_data[x]
                changed = True
        if changed:
            for x in self.auto_fields:
                self.entity[x] = self.auto_fields[x]()
            entity.to_yaml_file(settings.PITZ_DIR)
        return
    return type('entity_form', (forms.BaseForm,), { 'base_fields': fields, 'auto_fields': auto_fields, 'entity':entity, 'save': save })