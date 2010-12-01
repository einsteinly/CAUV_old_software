from django import forms
from tracker.track import models

class useruuid(forms.ModelForm):
    class Meta:
        model = models.UserProfile
        exclude = ('uuid')