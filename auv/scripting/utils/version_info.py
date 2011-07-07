import os

def getVersionInfo():
    repo_root = '/'.join(
        (os.path.join(os.getcwd(), __file__)).split('/')[:-5]
    )
    summary = os.system('hg -R %s summary --color=yes' % repo_root)
    diff = os.system('hg -R %s diff --color=yes' % repo_root)
    return 'summary:\n%s\ndiff:%s' % ()

print getVersionInfo()

