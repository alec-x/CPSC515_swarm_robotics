import matplotlib.pyplot as plt
import numpy as np
from robot import robot
from matplotlib.animation import FuncAnimation

num_bots = 5
robots = [robot(i,0) for i in range(num_bots)]

def animate_loop(i):
    ax.clear()

    for bot in robots:
        bot.update()
    x = [bot.x for bot in robots]
    y = [bot.y for bot in robots]
    ax.set_xlim(-1000,1000)
    ax.set_ylim(-1000,1000)
    ax.plot(x, y, 'ro')

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
# initialize plot
x = [bot.x for bot in robots]
y = [bot.y for bot in robots]
ax.set_xlim(-50,50)
ax.set_ylim(-50,50)
ax.plot(x, y, 'ro')

# create animation
animation = FuncAnimation(fig, func=animate_loop, interval=100)

# start animation
plt.show()