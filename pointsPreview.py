import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import os.path

obs_lines = []

def draw_obstacles(obstacles):
    for pol in obstacles:
        xs, ys = zip(*pol)  # create lists of x and y values
        plt.plot(xs, ys, 'brown')

def load_obs(data):
    res = []
    obs = [data[0]] + data[2:]
    for obs_str in obs:
        if len(obs_str.strip()) == 0:
            continue
        obs_data = obs_str.split()
        pol = []
        for x,y in zip(obs_data[1::2], obs_data[2::2]):
            pol.append([float(x), float(y)])

        pol.append(pol[0])  # repeat the first point to create a 'closed loop'
        res.append(pol)
    return res

def show_points(points_name, subplot_position, subplot_name):

    data = open(points_name, 'r').read()
    lines = data.split('\n')
    colors = ['black', 'gray', 'orange', 'green', 'pink', 'blue', 'red']
    points_types = [[] for _ in colors]
    for line in lines:
        if len(line.strip()) == 0:
            continue

        point_data = line.split()
        type = int(point_data[0])
        points_types[type].append([float(point_data[1]),float(point_data[2])])

    plt.subplot(subplot_position)
    plt.title(subplot_name)
    for i, point_type in enumerate(points_types):
        if i<present_level:
            continue

        for cp in point_type:
            plt.plot(cp[0],cp[1],color=colors[i], marker='o')

    lines = open(cmds[0], 'r').read().split('\n')
    obs = load_obs(lines)
    draw_obstacles(obs)

if len(sys.argv) < 2:
    print("[USAGE1] obs")
    exit()


cmds = sys.argv[1:]
present_level = 1
if len(cmds) > 1:
    present_level = int(cmds[1])

if os.path.isfile('enhanced_points'):
    plt.figure(figsize=(12,5))

    show_points('points', 121, "path finder")
    try:
        show_points('enhanced_points', 122, "path enhancer")
    except:
        print('enhanced points not exists')
else:
    show_points('points', 111, "path finder")

plt.show()
