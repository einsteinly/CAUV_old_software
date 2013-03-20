def subclassDict(cls_with_subs):
    classes = {}
    try:
        to_check = set(cls_with_subs.__subclasses__())
    except AttributeError:
        raise TypeError('Class must ultimately derive from object (new-style classes) not old style classes')
    checked = set()
    while len(to_check):
        cur = to_check.pop()
        checked.add(cur)
        if not getattr(cur, '_abstract', False):
            classes[cur.__name__] = cur
        for sub in cur.__subclasses__():
            if not sub in checked:
                to_check.add(sub)
    return classes
