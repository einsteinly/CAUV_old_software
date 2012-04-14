from AI_classes import aiProcess

from IPython.frontend.terminal.embed import InteractiveShellEmbed

ainode = aiProcess('ext_sender')

print """
To access AI use ainode, e.g.
-to change a condition, run ai.task_manager.notify_condition(condition_name, *args, **kwargs)
"""
ipshell = InteractiveShellEmbed()
ipshell()
