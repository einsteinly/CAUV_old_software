#!/bin/sh
rsync -rav . -P -e ssh cauv@10.0.0.3:/home/cauv/repos/software.hg --exclude='.hg' --exclude='pitzdir' --exclude-from=.hgignore
