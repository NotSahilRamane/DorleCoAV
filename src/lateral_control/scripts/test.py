import numpy as np
a=[100, 100,200,300,400,400]
a.sort()
for i in range(1,len(a)):
    if (a[i-1]==a[i]):
        print(a[i])