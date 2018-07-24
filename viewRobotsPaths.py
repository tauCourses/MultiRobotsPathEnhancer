import sys
import math
import numpy as np
from functools import partial
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Slider
from matplotlib import animation

def get_obs_lines(obstacles, ax):
    ret = []
    for pol in obstacles:
        xs, ys = zip(*pol)  # create lists of x and y
        temp, = ax.plot([], [], 'brown', lw=2)
        temp.set_data(xs, ys)
        ret.append(temp)

    return ret

def parse_file(filename):
    with open(filename, 'r') as myfile:
        return myfile.read().split('\n')
    
def parse_polygon(poly):
    data = poly.split()
    poly = [(float(point[0]), float(point[1])) for point in zip(data[1::2], data[2::2]) ]
    poly.append(poly[0])
    return poly

def parse_obstacles(data):
    obs = []
    poly = parse_polygon(data[0])
    if poly:
        obs.append(poly)
    for poly_str in data[2:]:
        poly = parse_polygon(poly_str)
        if poly:
            obs.append(poly)
    return obs
    
def parse_path(data):
    path = []

    for step in data[1:-1]:
        splited_step = step.split()
        path.append([(float(robot[0]), float(robot[1])) for robot in zip(splited_step[0::2], splited_step[1::2]) ])

    return path

def init(obs_lines):
    ret = obs_lines + [path_num_text]
    path_num_text.set_text('')
    return tuple(ret)


def animate(_, robots, path, frames_per_path_part, path_length, obs_lines, obstacles):
    """perform animation step"""
    global currentFrame, currentSpeed
    currentFrame = (currentFrame + currentSpeed) % (frames_per_path_part * (path_length - 1))
    index = int(currentFrame / frames_per_path_part)
    part_of = currentFrame % frames_per_path_part
    part_of = part_of / frames_per_path_part

    ret = [path_num_text]

    robots_points = path[index]
    next_robots_points = path[index + 1]
    for i, robot in enumerate(robots):
        temp_x = robots_points[i][0] + (next_robots_points[i][0] - robots_points[i][0])* part_of
        temp_y = robots_points[i][1] + (next_robots_points[i][1] - robots_points[i][1])* part_of
        robot.set_data([temp_x], [temp_y])
        ret.append(robot)

    for i in range(len(obstacles)):
        xs, ys = zip(*obstacles[i])  # create lists of x and y
        obs_lines[i].set_data(xs, ys)
        ret.append(obs_lines[i])

    path_num_text.set_text('path part = %d -> %d' % (index, index + 1))
    return tuple(ret)

if len(sys.argv) < 2:
    print("[USAGE1] obstacles.txt output.txt")
    exit()

cmds = sys.argv[1:]
obs = parse_obstacles(parse_file(cmds[0]))

output_file = 'output' if len(cmds) == 1 else cmds[1]
path = parse_path(parse_file(output_file))
number_of_robots = len(path[0])
path_length = len(path)

currentFrame = 0
currentSpeed = 20
fig = plt.figure()

ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-10, 10), ylim=(-10, 10))

obs_lines = get_obs_lines(obs, ax)
path_num_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
robots = [ax.plot([], [], 'o-', lw=2)[0] for _ in range(number_of_robots)]
frames_per_path_part = 300



animation_init = partial(init, obs_lines)

ani = animation.FuncAnimation(fig, animate, frames=frames_per_path_part * (path_length - 1),
                                fargs=(robots, path, frames_per_path_part, path_length, obs_lines, obs),
                                interval=33, blit=True,
                                init_func=animation_init)

axcolor = 'lightgoldenrodyellow'
plt.subplots_adjust(bottom=0.2)
axPlayer = plt.axes([0.1, 0.1, 0.8, 0.025], facecolor=axcolor)
axSpeed = plt.axes([0.1, 0.05, 0.8, 0.025], facecolor=axcolor)

def sliderChange(val):
    global currentFrame
    currentFrame = val


def sliderChangeSpeed(val):
    global currentSpeed
    currentSpeed = val

splayer = Slider(axPlayer, 'Player', 0, frames_per_path_part * (path_length - 1), valinit=currentFrame)
splayerSpeed = Slider(axSpeed, 'Speed', 0, 100, valinit=currentSpeed)
splayer.on_changed(sliderChange)
splayerSpeed.on_changed(sliderChangeSpeed)

plt.show()
