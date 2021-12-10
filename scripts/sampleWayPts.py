# sample waypoints from numpy array
import os
import numpy as np
import matplotlib.pyplot as plt

trajX = []
trajY = []

path = '/home/deepak/IIITD/catkin_ws/src/floam/scripts'

outputX = 'wayptsX_2.txt'
outputY = 'wayptsY_2.txt'

inputFile = 'demo_10hz.npy'

data = np.load(path + '/' + inputFile)

fX = open(outputX,"w")
fY = open(outputY,"w")

for i in range(0,data.shape[0],80):
    if i is not 0 and data[i,1] < 1:
        continue
    
    x = data[i,1]
    y = data[i,2]

    print(f'X and Y are: {[x,y]}')
    fX.write(str(x))
    fY.write(str(y))

    trajX.append(x)
    trajY.append(y)

fX.close()
fY.close()

print(f'size of the waypoints file is: {len(trajX)}')
plt.plot(trajX, trajY)
plt.show()