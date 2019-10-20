#!/usr/bin/python
import sys

output = "["
with open(sys.argv[1], 'r') as f:
    lines = f.readlines()
    for i in range (3, len(lines)):
        data = lines[i]
        data = data.replace('{', '[')
        data = data.replace('}', ']')
        data = data.replace(';', '')
        output = output + data

output = eval(output)

with open(sys.argv[1] + 'l', 'wb') as f:
    i = 0
    for byte in output:
        #print(chr(byte))
        f.write(chr(byte))
        i = i + 1
        #if i % 40 == 0:
            #f.write('\n')
