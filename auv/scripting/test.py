import traceback

import messages
import ssrc.spread
import struct

def ords(bs):
	r = ''
	for b in bs:
		r += '%02x' % ord(b) + ' '
	r = r.rstrip(' ')
	return r

def pg(grouplist):
    r = ""
    for i in xrange(0, grouplist.size()):
        r += str(grouplist.group(i)) + (",","")[i==grouplist.size()-1]
    return r

try:
    mb = ssrc.spread.Mailbox("16707@localhost", "", True, ssrc.spread.Timeout(3))
    mb.join("image")
    mb.join("pipeline")
    mb.join("pl_gui")
    
    # default message capacity (normally 4096) is defined in Message.h:85
    # msg = ssrc.spread.Message(10000) # non-default capacity
    msg = ssrc.spread.Message()
    grouplist = ssrc.spread.GroupList() # default capacity is 10 groups
    
    i = 0
    while True:
        mb.receive(msg, grouplist)
        #msgbytes = msg.read(msg.size())
        msg.seek(0)
        # TODO: fix ssrcspread SIWG API: for some reason msg.read will only ever return one byte at a time...
        msgbytes = ''
        for i in xrange(0, msg.size()):
            msgbytes += msg.read(1)
        print i,
        if msg.is_regular():
            print 'Regular message received:',
        elif msg.is_membership():
            print 'Membership message received:',
        else:
            print 'Some sort of weird unknown message received:',
        print '"%s" (%d), sent to: %s' % (
                  msgbytes,
                  len(msgbytes),
                  pg(grouplist)
                 )
        print "Message bytes:", ords(msgbytes)

        msg.clear()
        i += 1

except Exception, e:
    traceback.print_exc()

