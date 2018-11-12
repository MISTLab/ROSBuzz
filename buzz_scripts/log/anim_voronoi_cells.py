#!/usr/bin/python
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')
import matplotlib.animation as animation
import numpy as np
import csv

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
fig.set_tight_layout(True)
axes = plt.gca()
axes.set_xlim([-70,70])
axes.set_ylim([-70,70])
datafile = open('src/rosbuzz/buzz_scripts/log/voronoi_3.csv', 'r')
Vorreader = csv.reader(datafile, delimiter=',')

def animate(i):
    for row in Vorreader:
        ax.clear()
#        ax.plot([-50, -50, 50, 50, -50],[-50, 50, 50, -50, -50],'b--')
        j = 1
        while j < len(row)-2:
            ax.plot([float(row[j]), float(row[j+2])], [float(row[j+1]), float(row[j+3])])
            j += 6
        ax.plot(float(row[len(row)-2]),float(row[len(row)-1]),'x')
        return

ani = animation.FuncAnimation(fig, animate, interval=250)
ani.save('Voronoi.gif', dpi=80, writer='imagemagick')
plt.show()
