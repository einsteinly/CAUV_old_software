import os

def getVersionInfo():
    repo_root = '/'.join(
        (os.path.join(os.getcwd(), __file__)).split('/')[:-5]
    )
    (si, so) = os.popen2('hg -R %s summary --color=yes' % repo_root)
    summary = so.read()
    (si, so) = os.popen2('hg -R %s diff --color=yes' % repo_root)
    diff = so.read()
    return (summary, diff)


