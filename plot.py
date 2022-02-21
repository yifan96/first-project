
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rosbag
from matplotlib.patches import Circle, PathPatch
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D
import mpl_toolkits.mplot3d.art3d as art3d

def data_for_cylinder_along_z(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 50)
    theta = np.linspace(0, 2*np.pi, 50)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid



bag = rosbag.Bag('/home/yifan/4agent_planning.bag')
traj1 = []
traj2 = []
traj3 = []
traj4 = []

for topic, msg , t in bag.read_messages(topics='/hummingbird1/ground_truth/position'):
    traj1.append([msg.point.x,msg.point.y,msg.point.z])
for topic, msg , t in bag.read_messages(topics='/hummingbird2/ground_truth/position'):
    traj2.append([msg.point.x,msg.point.y,msg.point.z])
for topic, msg , t in bag.read_messages(topics='/hummingbird3/ground_truth/position'):
    traj3.append([msg.point.x,msg.point.y,msg.point.z])
for topic, msg , t in bag.read_messages(topics='/hummingbird4/ground_truth/position'):
    traj4.append([msg.point.x,msg.point.y,msg.point.z])
bag.close()


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')

ax.plot(np.array(traj1)[:,0],np.array(traj1)[:,1],np.array(traj1)[:,2], label='agent 1')
ax.plot(np.array(traj2)[:,0],np.array(traj2)[:,1],np.array(traj2)[:,2], label='agent 2')
ax.plot(np.array(traj3)[:,0],np.array(traj3)[:,1],np.array(traj3)[:,2], label='agent 3')
ax.plot(np.array(traj4)[:,0],np.array(traj4)[:,1],np.array(traj4)[:,2], label='agent 4')
ax.legend(loc="upper right", prop={'size': 13})

#ax.plot([3.8, 1.23], [-1.1, -1], [0, 2.3], '--', color='dimgrey')

Xm1,Ym1,Zm1 = data_for_cylinder_along_z(3.8,-5.3,0.5,3.5)
ax.plot_surface(Xm1, Ym1, Zm1, alpha=1,color='darkred')
p = Circle((3.8, -5.3), 0.5, color='darkred')
p_d = Circle((3.8, -5.3), 0.5, color='darkred')
ax.add_patch(p)
art3d.pathpatch_2d_to_3d(p, z=3.5, zdir="z")
ax.add_patch(p_d)
art3d.pathpatch_2d_to_3d(p_d, z=0, zdir="z")

Xm2,Ym2,Zm2 = data_for_cylinder_along_z(0,0,0.5,3.5)
ax.plot_surface(Xm2, Ym2, Zm2, alpha=1, color='purple')
p1 = Circle((0, 0), 0.5, alpha=1, color='purple')
ax.add_patch(p1)
art3d.pathpatch_2d_to_3d(p1, z=3.5, zdir="z")
p1_d = Circle((0, 0), 0.5, alpha=1, color='purple')
ax.add_patch(p1_d)
art3d.pathpatch_2d_to_3d(p1_d, z=0, zdir="z")


Xl1,Yl1,Zl1 = data_for_cylinder_along_z(-3.7,3.3,1,3.5)
ax.plot_surface(Xl1, Yl1, Zl1, alpha=1, color='green')
p2 = Circle((-3.7, 3.3), 1, color='green')
ax.add_patch(p2)
art3d.pathpatch_2d_to_3d(p2, z=3.5, zdir="z")
p2_d = Circle((-3.7, 3.3), 1, color='green')
ax.add_patch(p2_d)
art3d.pathpatch_2d_to_3d(p2_d, z=0, zdir="z")

Xs,Ys,Zs = data_for_cylinder_along_z(2.2,-2.4,0.3,3.5)
ax.plot_surface(Xs, Ys, Zs, alpha=1, color='goldenrod')
p3 = Circle((2.2, -2.4), 0.3, color='goldenrod')
ax.add_patch(p3)
art3d.pathpatch_2d_to_3d(p3, z=3.5, zdir="z")
p3_d = Circle((2.2, -2.4), 0.3, color='goldenrod')
ax.add_patch(p3_d)
art3d.pathpatch_2d_to_3d(p3_d, z=0, zdir="z")


#box
ax.bar3d(4.3,2.8,0,2,2,3.5)
ax.bar3d(-6.2,-5.5,0,3,3,2)
#wall

ax.bar3d(-9,-9,0,18.15,0.15,3.5,color='w')
ax.bar3d(-9,9,0,18.15,0.15,3.5,color='w')
ax.bar3d(-9,-9,0,0.15,18.15,3.5,color='w')
ax.bar3d(9,-9,0,0.15,18.15,3.5,color='w')
ax.set_xlabel('x-axis(m)')
ax.set_ylabel('y-axis(m)')
ax.set_zlabel('z-axis(m)')
# first trial
ax.scatter([0,1, 2, 3],[3.5,3.5,3.5,3.5],[0,0,0,0], c='k', marker='o')
ax.scatter([-4.5,-7.5, 6.5, 8],[-7,3.5,-5,8],[1.5,2.3,1,0.8], c='r', marker='*', s=50)
# second trial
# ax.scatter([3.8, -3.1, -1.3, -6.5],[-1.1, -0.37, 2.8, 2.0],[0,0,0,0], c='k', marker='o')
# ax.scatter([7.7, 1.23, 1.48, -3.4],[-5.5, -1.0 , 6.22, 7.3],[1.5, 2.3, 1.0 ,0.8], c='r', marker='*', s=50)
plt.gca().set_box_aspect((18, 18, 3.5))
plt.show()
