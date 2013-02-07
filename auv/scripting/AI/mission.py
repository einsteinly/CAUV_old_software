import yaml
import options
import conditions
import tasks
import detector_library
import script_library

def task_representer(dumper, task):
    return dumper.represent_mapping("!task", {
        "name" : task.name,
        "script" : task.script,
        "options" : task.options.to_dict(),
        "conditions" : task.conditions,
    })

def task_constructor(loader, data):
    mapping = loader.construct_mapping(data, deep=True)
    return tasks.Task(mapping['conditions'],
                      mapping['script'],
                      mapping['options'],
                      name = mapping['name'])

def script_representer(dumper, script):
    return dumper.represent_mapping("!script/{}".format(script.name),
                                    script.options.to_dict())

def script_constructor(loader, tag_suffix, data):
    return tasks.TaskScript(tag_suffix, loader.construct_mapping(data))

def condition_representer(dumper, condition):
    return dumper.represent_mapping("!condition/{}".format(condition.get_type()),
                                    {"name" : condition.name,
                                     "options": condition.options.to_dict()})

def condition_constructor(loader, tag_suffix, data):
    mapping = loader.construct_mapping(data, deep=True)
    condition_class = conditions.get_conditions()[tag_suffix]
    return condition_class(loader._ai_state, mapping['options'], mapping['name'])

yaml.add_representer(tasks.Task, task_representer)
yaml.add_constructor("!task", task_constructor)
yaml.add_representer(tasks.TaskScript, script_representer)
yaml.add_multi_constructor("!script/", script_constructor)
yaml.add_multi_representer(conditions.Condition, condition_representer)
yaml.add_multi_constructor("!condition/", condition_constructor)

dump = yaml.dump
def load(stream, ai_state=None):
    loader = yaml.Loader(stream)
    loader._ai_state = ai_state
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()

def load_state(stream, tasks):
    options = yaml.load(stream)
    for task, state_dict in options.iteritems():
        try:
            tasks[task].state.from_dict(state_dict)
        except KeyError:
            warning("Task {} does not exist!".format(task))

def dump_state(tasks, stream = None):
    options = {t: v.state.to_dict() for t, v in tasks.items()}
    return yaml.dump(options, stream)
