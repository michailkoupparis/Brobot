#!/usr/bin/env python3
xfile = open('x.rtf', 'w')
xs =  set(["Postcard", "Radio", "Telegram"])
for x in xs:
    xfile.write(x+'\n')

xfile.close()
