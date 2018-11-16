#!/usr/bin/env python3
import os.path
xfile = open('x.rtf', 'w')
xs =  set(["Postcard", "Radio", "Telegram"])
for x in xs:
    xfile.write(x+'\n')

xfile.close()

xfile = open('x.rtf','r+')
xs =  xfile.readlines()
i =0
for x in xs:
    print(i, 'line :', x)
    i = i+1

xfile.write('Television'+'\n')
print('38224380',os.path.exists("/home/robot/x.rtf"))

xfile.close()
xfile = open('x.rtf')
xs =  xfile.readlines()
i =0
for x in xs:
    print(i, 'line :', x)
    i = i+1

xfile.close()
