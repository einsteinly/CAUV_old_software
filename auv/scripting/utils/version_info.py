import os

def getVersionInfo():
    return os.system('hg -R %s summary --color=yes' % os.path.join(os.getcwd(), __file__))

print getVersionInfo()

