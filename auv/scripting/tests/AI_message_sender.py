from AI_classes import aiProcess

from IPython.Shell import IPShellEmbed

ainode = aiProcess('ext_sender')

print """
To access AI use ainode, e.g.
-to change a condition, run ai.task_manager.notify_condition(condition_name, *args, **kwargs)
"""
ipshell = IPShellEmbed()
ipshell()
