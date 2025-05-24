#!/usr/bin/env python3
# -*- coding:utf-8 -*-
from time import *
from math import *

snail = 0
up = 5
down = 3
distance = 1000000000000000000000000000000
day = 0  # result

start_time = time()
day = ceil((distance - down) / (up - down))
end_time = time()

print(day)
print(end_time - start_time)
