#!/usr/bin/env python3

# primary reason for python 3 is support for %z in srtptime

import subprocess 
import collections
import datetime
import os

branch = 'default'
log_proc = subprocess.Popen(['hg','log','--branch',branch],stdout = subprocess.PIPE)
hg_log,dummy = log_proc.communicate()
hg_log = str(hg_log,encoding='ascii')
hg_log = hg_log.split('\n')

class Revision:
    def __init__(self):
        self.date = None
        self.revision_num = None
        self.revision_sha1 = None
        self.user = None
        self.summary = None

    def parse_changeset(self,string):
        self.revision_num,self.revision_sha1 = string.split(':',1)

    def parse_summary(self,string):
        self.summary = string
    
    def parse_date(self,string):
        self.date = datetime.datetime.strptime(string,'%a %b %d %H:%M:%S %Y %z')

    def parse_user(self,string):
        self.user = string

    def is_valid(self):
        return self.date is not None and self.revision_num is not None and self.user is not None and self.summary is not None

parse_map = collections.defaultdict (
lambda: lambda x,y: None,
{
    'changeset': Revision.parse_changeset,
    'summary': Revision.parse_summary,
    'date': Revision.parse_date,
    'user': Revision.parse_user
})

revisions = []

current_rev = Revision()

for line in (x.strip() for x in hg_log):
    if not line:
        revisions.append(current_rev)
        current_rev = Revision()
        continue
    info_id, info = line.split(':',1)
    parse_map[info_id.strip()](current_rev,info.strip())

revisions = [r for r in revisions if r.is_valid()]
current_revision = revisions[0]

entry = """cauv-software ({version}-1) unstable; urgency=low

  * {summary}

 -- {name}  {date}
"""

with open('debian/changelog','w') as changelog_file:
    for rev in revisions:
        changelog_file.write(entry.format(
            version = rev.revision_num,
            summary = rev.summary,
            name = rev.user,
            date = rev.date.strftime('%a, %d %b %Y %H:%M:%S %z')
        ))

orig_tar_file = '../cauv-software_{0}.orig.tar.gz'.format(current_revision.revision_num)
print('creating',orig_tar_file,'...')
tar_proc = subprocess.Popen(['tar','--exclude','debian/*','-czf',orig_tar_file,'.'])
tar_proc.wait()

print('running dpkg-buildpackage')
dpkg_env = dict(os.environ)
dpkg_env['DEB_BUILD_OPTIONS'] = 'parallel=4 nocheck'
dpkg_proc = subprocess.Popen(['dpkg-buildpackage','-us','-uc','-j4'])
dpkg_proc.wait()

print('done!')
