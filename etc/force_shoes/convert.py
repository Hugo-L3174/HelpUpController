#!/usr/bin/env python3

import csv
import yaml

fin = open('/home/hugo/LIRMM/HelpUpController/etc/force_shoes/MT_00131752_002-003.txt')
in_ = csv.reader(filter(lambda row: not row.startswith('//'), fin), delimiter = '\t')
header = next(in_)
data = {h: [] for h in header if len(h)}
for row in in_:
    for i, value in enumerate(row):
        if header[i] in data:
            data[header[i]].append(value)
out = open('data.yaml', 'w')
out.write(yaml.dump(data))

# careful: this code appends the yaml file in alphabetic order for the headers
