import traceback

import messages
import ssrc.spread

try:
    mb = ssrc.spread.Mailbox("16707@localhost", "", True)
    mb.join("image")
    mb.join("pipeline")
    mb.join("pl_gui")
    
    print mb.receive()
except Exception, e:
    traceback.print_exc()

print "hello, AUV"

