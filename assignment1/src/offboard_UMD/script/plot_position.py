import numpy as np
import rosbag
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_quaternion

# Read bag file
bag = rosbag.Bag('2021-09-21-19-19-58.bag')

x = []
y = []
z = []
roll = []
pitch = []
yaw = []
time = []
cycles = []
cycle_time = []

init_time = 0

for topic, msg, t in bag.read_messages(topics=['/mavros/local_position/pose', '/mavros/path_cycle']):
    if topic == '/mavros/local_position/pose':
        current_time = t.to_sec()
        x.append(msg.pose.position.x)
        y.append(msg.pose.position.y)
        z.append(msg.pose.position.z)

        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (r, p, ya) = euler_from_quaternion (orientation_list)

        roll.append(r)
        pitch.append(p)
        yaw.append(ya)

        if init_time == 0:
            time.append(0)
            init_time = current_time
        else:
            time.append(current_time - init_time)
    else:
        cycles.append(msg.cycle)
        cycle_time.append(t.to_sec() - init_time)

data = np.stack((x, y, z, roll, pitch, yaw, time))

cycles.append(5)
cycle_step = 0
cycle_data = {}
past_idx = 0

for idx, tim in enumerate(time):
    if cycle_time[cycle_step] < tim:
        cycle_data['cycle_{}'.format(cycle_step)] = data[:, past_idx:idx]
        cycle_step += 1
        past_idx = idx
        if cycle_step > 4:
            cycle_data['cycle_{}'.format(cycle_step)] = data[:, idx+1:]
            break

## Plot position ##
##################

fig1, ax1 = plt.subplots(figsize=(20,20))

ax1.set_ylim([-6, 31])
ax1.plot(time, x, linewidth=2.5, label='x')
ax1.plot(time, y, linewidth=2.5, label='y')
ax1.plot(time, z, linewidth=2.5, label='z')
ax1.set_title("XYZ Position", fontweight = 'heavy')
ax1.set(xlabel="Time [s]", ylabel="Distance [m]")
ax1.legend(shadow=True, fancybox=True, loc='upper right')

## Plot orientation ##
######################

fig2, ax2 = plt.subplots(figsize=(20,20))

ax2.set_ylim([-1, 2.5])
ax2.plot(time, roll, linewidth=2.5, label='roll')
ax2.plot(time, pitch, linewidth=2.5, label='pitch')
ax2.plot(time, yaw, linewidth=2.5, label='yaw')
ax2.set_title("RPY Orientation", fontweight = 'heavy')
ax2.set(xlabel="Time [s]", ylabel="Angle [rad]")
ax2.legend(shadow=True, fancybox=True, loc='upper right')

last_tim = 0
for c, tim in enumerate(cycle_time):
    ax1.axvline(x=tim, color='k', linestyle='--', alpha=0.4)
    ax1.annotate(s='', xy=(last_tim,28), xytext=(tim,28), arrowprops=dict(arrowstyle='<->'))

    ax2.axvline(x=tim, color='k', linestyle='--', alpha=0.4)
    ax2.annotate(s='', xy=(last_tim,2.1), xytext=(tim,2.1), arrowprops=dict(arrowstyle='<->'))

    if c == 0:
        l = "Takeoff"
    else:
        l = "Cycle {}".format(c)

    ax1.text((tim-last_tim)/2 + last_tim, 29.5, l, horizontalalignment='center',
        verticalalignment='center', weight='bold')

    ax2.text((tim-last_tim)/2 + last_tim, 2.2, l, horizontalalignment='center',
        verticalalignment='center', weight='bold')

    
    last_tim = tim

ax1.annotate(s='', xy=(last_tim,28), xytext=(data[6, -1],28), arrowprops=dict(arrowstyle='<->'))
ax1.text((data[6, -1]-last_tim)/2 + last_tim, 29.5, 'Landing', horizontalalignment='left',
        verticalalignment='center', fontsize=10, weight='bold')

ax2.annotate(s='', xy=(last_tim,2.1), xytext=(data[6, -1],2.1), arrowprops=dict(arrowstyle='<->'))
ax2.text((data[6, -1]-last_tim)/2 + last_tim, 2.2, 'Landing', horizontalalignment='left',
        verticalalignment='center', fontsize=10, weight='bold')


## Position 3D plot ##
######################

xs = [0, 0, 10, 10, 10]
ys = [0, 0, 0, 0, 5]
zs = [0, 10, 10, 25, 25]
fig3 = plt.figure(figsize=(20, 20))
ax3 = Axes3D(fig3, alpha=0.1)
ax3.set_title("3D XYZ Trajectory", fontweight = 'heavy')

for c in cycles:
    data = cycle_data['cycle_{}'.format(c)]
    if c > 0 and c < 5:
        l = 'cycle_{}'.format(c)
    elif c == 0:
        l = 'takeoff'
    else:
        l = 'landing'
    ax3.plot3D(data[0, :], data[1, :], data[2, :], label=l, linewidth=2.5)
ax3.legend(shadow=True, fancybox=True)
ax3.scatter(xs, ys, zs, s=35, c='k')
for xt, yt, zt in zip(xs, ys, zs):
    ax3.text3D(xt + 0.1, yt + 0.1, zt + 0.1, '({},{},{})'.format(xt, yt, zt), 
    fontsize=10, fontweight = 'heavy')
ax3.set(xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]")

## Plot trajectories in X-Y X-Z & Y-Z planes ##
###############################################

fig4 = plt.figure(figsize=(20,20))
ax4 = fig4.add_subplot(131)
ax5 = fig4.add_subplot(132)
ax6 = fig4.add_subplot(133)

for c in cycles:
    data = cycle_data['cycle_{}'.format(c)]
    if c > 0 and c < 5:
        l = 'cycle_{}'.format(c)
    elif c == 0:
        l = 'takeoff'
    else:
        l = 'landing'
    ax4.plot(data[0, :], data[1, :], label=l, linewidth=2.5)
    ax5.plot(data[0, :], data[2, :], label=l, linewidth=2.5)
    ax6.plot(data[1, :], data[2, :], label=l, linewidth=2.5)

ax4.set_title("Trajectory XY", fontweight = 'heavy')
ax4.set(xlabel="X [m]", ylabel="Y [m]")
ax4.legend(shadow=True, fancybox=True, loc='upper left')

ax5.set_title("Trajectory XZ", fontweight = 'heavy')
ax5.set(xlabel="X [m]", ylabel="Z [m]")
ax5.legend(shadow=True, fancybox=True, loc='lower right')

ax6.set_title("Trajectory YZ", fontweight = 'heavy')
ax6.set(xlabel="Y [m]", ylabel="Z [m]")
ax6.legend(shadow=True, fancybox=True, loc='lower right')

for xt, yt, zt in zip(xs, ys, zs):
    ax4.text(xt + 0.2, yt + 0.2,  '({},{})'.format(xt, yt), 
    fontsize=10, fontweight = 'heavy')
    ax5.text(xt + 0.2, zt + 0.2,  '({},{})'.format(xt, zt), 
    fontsize=10, fontweight = 'heavy')
    ax6.text(yt + 0.2, zt + 0.2,  '({},{})'.format(yt, zt), 
    fontsize=10, fontweight = 'heavy')

plt.show()