from django.db import models
from django.contrib.auth.models import User as djangoUser

class UserProfile(models.Model):
    user = models.OneToOneField(djangoUser)
    uuid = models.CharField(max_length=10) # stores user's pitz uuid
    def __unicode__(self):
        return self.user.username
    
# Create your models here.
