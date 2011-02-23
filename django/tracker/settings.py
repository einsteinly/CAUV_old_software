import os
# Django settings for tracker project.
# location of tracker project (used to to make some relative paths absolute)
SITE_ROOT = os.path.dirname(os.path.realpath(__file__)) 
AUTH_PROFILE_MODULE = 'track.UserProfile' #tells django to link user profile to django auth users
PITZ_DIR = os.sep.join([os.sep.join(SITE_ROOT.split(os.sep)[0:-2]), 'pitzdir']) #pitzdir
PITZ_ATTACHED_FILES_DIR = os.sep.join([SITE_ROOT, 'static', 'attached_files'])

if "SCRIPT_NAME" in os.environ:
    ROOT_URL = os.environ["SCRIPT_NAME"]
else:
    ROOT_URL = ""

LOGIN_REDIRECT_URL = ROOT_URL+"/"
LOGIN_URL = ROOT_URL+"/accounts/login"
LOGOUT_URL = ROOT_URL+"/accounts/logout"
SESSION_COOKIE_PATH = "/"
ATTACHED_FILES_URL = ROOT_URL+"/static/attached_files/"

# URL that handles the media served from MEDIA_ROOT. Make sure to use a
# trailing slash if there is a path component (optional in other cases).
# Examples: "http://media.lawrence.com", "http://example.com/media/"
MEDIA_URL = ROOT_URL + '/static/'
STATIC_URL = ROOT_URL + "/static/"


DEBUG = True
TEMPLATE_DEBUG = DEBUG

ADMINS = (
    # ('Your Name', 'your_email@domain.com'),
)

MANAGERS = ADMINS

DATABASE_ENGINE = 'django.db.backends.sqlite3'           # 'postgresql_psycopg2', 'postgresql', 'mysql', 'sqlite3' or 'oracle'.
DATABASE_NAME = SITE_ROOT+'/db/db.db'             # Or path to database file if using sqlite3.
DATABASE_USER = ''             # Not used with sqlite3.
DATABASE_PASSWORD = ''         # Not used with sqlite3.
DATABASE_HOST = ''             # Set to empty string for localhost. Not used with sqlite3.
DATABASE_PORT = ''             # Set to empty string for default. Not used with sqlite3.

# Local time zone for this installation. Choices can be found here:
# http://en.wikipedia.org/wiki/List_of_tz_zones_by_name
# although not all choices may be available on all operating systems.
# If running in a Windows environment this must be set to the same as your
# system time zone.
TIME_ZONE = 'Europe/London'

# Language code for this installation. All choices can be found here:
# http://www.i18nguy.com/unicode/language-identifiers.html
LANGUAGE_CODE = 'en-gb'

SITE_ID = 1

# If you set this to False, Django will make some optimizations so as not
# to load the internationalization machinery.
USE_I18N = True

# Absolute path to the directory that holds media.
# Example: "/home/media/media.lawrence.com/"
MEDIA_ROOT = SITE_ROOT+'/static/'

# URL prefix for admin media -- CSS, JavaScript and images. Make sure to use a
# trailing slash.
# Examples: "http://foo.com/media/", "/media/".
ADMIN_MEDIA_PREFIX = ROOT_URL+'/static/adminmedia/'

# Make this unique, and don't share it with anybody.
SECRET_KEY = '%@p%d#%65td-&xbviu348dx90=x&zlkw4wls$8+_f$3ca4n%v2'

# List of callables that know how to import templates from various sources.
TEMPLATE_LOADERS = (
    'django.template.loaders.filesystem.load_template_source',
    'django.template.loaders.app_directories.load_template_source',
#     'django.template.loaders.eggs.load_template_source',
)

from django.conf.global_settings import TEMPLATE_CONTEXT_PROCESSORS
TEMPLATE_CONTEXT_PROCESSORS += (
    'django.core.context_processors.request',
    'django.core.context_processors.i18n',
    'tracker.track.context_processors.root_url',
)

MIDDLEWARE_CLASSES = (
    'django.middleware.common.CommonMiddleware',
    'django.contrib.sessions.middleware.SessionMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware',
    #'django.contrib.csrf.middleware.CsrfViewMiddleware',
)

ROOT_URLCONF = 'tracker.urls'

TEMPLATE_DIRS = (
    SITE_ROOT+"/templates"
    # Put strings here, like "/home/html/django_templates" or "C:/www/django/templates".
    # Always use forward slashes, even on Windows.
    # Don't forget to use absolute paths, not relative paths.
)

#ONLY USE FOR DEBUGGING TEMPLATES (BREAKS SOME THINGS)
#TEMPLATE_STRING_IF_INVALID = 'Oops, something went wrong here, ref %s'

CACHE_BACKEND = 'locmem://'

INSTALLED_APPS = (
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.sites',
    'django.contrib.admin',
    'django.contrib.markup',
    'tracker.track'
)
