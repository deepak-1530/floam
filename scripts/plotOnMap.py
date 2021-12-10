# plot floam trajectory with GPS waypoints
import numpy as np
import matplotlib.pyplot as plt
import tilemapbase
tilemapbase.start_logging()
tilemapbase.init(create=True)

# Use open street map
t = tilemapbase.tiles.build_OSM()

# read the numpy array
gnssData = np.load('/home/deepak/IIITD/catkin_ws/src/floamBag/Demo_Gnss.npy')

data = []

for i in range(gnssData.shape[0]):
    data.append((gnssData[i,0], gnssData[i,1]))

# Feed the starting location of the car -> 

# My current office at the University of Leeds
start = (gnssData[100,0], gnssData[100,1])
print(f'Start is: {start}')
#pts = [(-1.554934, 53.804198), (-1.554944, 53.804178)]

degree_range = 0.3
extent = tilemapbase.Extent.from_lonlat(start[0] - degree_range, start[0] + degree_range,
                  start[1] - degree_range, start[1] + degree_range)
extent = extent.to_aspect(1.0)
print(extent)



# On my desktop, DPI gets scaled by 0.75
fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
ax.xaxis.set_visible(True)
ax.yaxis.set_visible(True)

plotter = tilemapbase.Plotter(extent, t, width=600)
plotter.plot(ax, t)

ptsX = []
ptsY = []

for i in range(len(data)):
    x, y = tilemapbase.project(*data[i])
    ptsX.append(x)
    ptsY.append(y)
    ax.scatter(x,y, marker=".", color="black", linewidth=20)
#ax.plot(x,y,linewidth=2)


plt.show()

'''
extent1 = extent.with_centre_lonlat(-1.558, 53.804198)
plotter2 = tilemapbase.Plotter(extent1, t, width=600)

fig, ax = plt.subplots(figsize=(8, 8), dpi=100)
plotter2.plot(ax)
plt.show()
'''
