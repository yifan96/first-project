#!/usr/bin/env python3
 # license removed for brevity
import rospkg
import rospy
import opengen as og
import numpy
from std_msgs.msg import String
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
#from euler_to_quaternion import euler_to_quaternion
import statistics
from mav_msgs.msg import RollPitchYawrateThrust
from std_msgs.msg import Float64
from traj_msg.msg import OptimizationResult
import time
import sys
from quaternion_to_euler import quaternion_to_euler
import math
from adapt_weights import adapt_weights
mng = og.tcp.OptimizerTcpManager('/home/yifan/catkin_ws/src/potentialfield3d/src/MAV/shafter_nmpc_dist_1')
mng.start()
xpos = 0
ypos = 0
zpos = 0
qx = 0
qy = 0
qz = 0
qw = 0
vx = 0
vy = 0
vz = 0
k = 0
roll = 0
pitch = 0
yaw = 0
roll_v = 0
pitch_v = 0
yaw_v = 0
yawrate = 0
t0 = 8
C = 9.81 / t0
#obsdata = [0]*(3)
N = 20 # achilleas horizon (change horizon on optimization as well)
ustar = [9.81,0.0,0.0] * (N)

xobs1_b = [100]*(3*N)
xobs2_b = [100]*(3*N)
xobs1_g = [100]*(3*N)
xobs2_g = [100]*(3*N)

nu = 3
dt = 1.0/20
x0 = [0,0,0.0,0.0,0.0,0.0,0.0,0.0]
xref = [0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
global uold
uold = [9.81,0.0,0.0]
uref = [9.81,0.0,0.0]

v_z = [0.0, 0.0, 0.0]
p_f = [0,0,0]
f_nmhe = [0,0,0]
land_flag = 0
start_flag = 0
safety_counter = 0
path = rospy.get_param('traj/traj_path')
traj = numpy.loadtxt(path)
rospy.loginfo(path)

def global2body(xobs_g, yaw):
    xobs_b = [100]*(3*N)
    for i in range(0,N):
        xobs_b[3*i]= math.cos(yaw)*xobs_g[3*i] + math.sin(yaw)*xobs_g[3*i+1]
        xobs_b[3*i+1]= -math.sin(yaw)*xobs_g[3*i] + math.cos(yaw)*xobs_g[3*i+1]
        xobs_b[3*i+2]= xobs_g[3*i+2]
    return xobs_b

def body2global(xobs_b, yaw):
    xobs_g = [100]*(3*N)
    for i in range(0,N):
        xobs_g[3*i]= math.cos(yaw)*xobs_b[3*i] - math.sin(yaw)*xobs_b[3*i+1]
        xobs_g[3*i+1]= math.sin(yaw)*xobs_b[3*i] + math.cos(yaw)*xobs_b[3*i+1]
        xobs_g[3*i+2]= xobs_b[3*i+2]
    return xobs_g

def predict(x, u):

    N = 40
    p_hist = []
    v_hist = []
    v_hist = v_hist + list(x[3:6])
    x_hist = [0.0]*N*8
    x_hist[0:8] = x
    dt = 1.0 / 20
    for i in range(1,N):
        x_hist[8*i] = x_hist[8*(i-1)] + dt * x_hist[8*(i-1)+3]
        x_hist[8*i+1] = x_hist[8*(i-1)+1] + dt * x_hist[8*(i-1)+4] 
        x_hist[8*i+2] = x_hist[8*(i-1)+2] + dt * x_hist[8*(i-1)+5] 
        x_hist[8*i+3] = x_hist[8*(i-1)+3] + dt * (math.sin(x_hist[8*(i-1)+7]) * math.cos(x_hist[8*(i-1)+6]) * u[3*(i-1)] - 0.1 * x_hist[8*(i-1)+3]) 
        x_hist[8*i+4] = x_hist[8*(i-1)+4] + dt * (-math.sin(x_hist[8*(i-1)+6]) * u[3*(i-1)] - 0.1*x_hist[8*(i-1)+4])
        x_hist[8*i+5] = x_hist[8*(i-1)+5] + dt * (math.cos(x_hist[8*(i-1)+7]) * math.cos(x_hist[8*(i-1)+6]) * u[3*(i-1)] - 0.2 * x_hist[8*(i-1)+5] - 9.81) 
        x_hist[8*i+6] = x_hist[8*(i-1)+6] + dt * ((1 / 0.20) * (u[3*(i-1)+1] - x_hist[8*(i-1)+6])) 
        x_hist[8*i+7] = x_hist[8*(i-1)+7] + dt * ((1 / 0.17) * (u[3*(i-1)+2] - x_hist[8*(i-1)+7]))
       
        p_hist = p_hist + [x_hist[8*i],x_hist[8*i+1],x_hist[8*i+2]]
        v_hist = v_hist + [x_hist[8*i+3],x_hist[8*i+4],x_hist[8*i+5]]
    p_next_x = x_hist[8*(N-1)] + dt*x_hist[8*(N-1)+3]
    p_next_y = x_hist[8*(N-1)+1] + dt*x_hist[8*(N-1)+4]
    p_next_z = x_hist[8*(N-1)+2] + dt*x_hist[8*(N-1)+5]
    p_hist = p_hist + [p_next_x] + [p_next_y]+ [p_next_z]
    return(p_hist)

def quaternion_to_euler(x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [roll, pitch, yaw]

def adapt_weights(forces):
    c = 1
    fsum = abs(forces[0]) + abs(forces[1]) + abs(forces[2])
    scale = 1 / (1 + c*fsum)

    return scale


def callback_pot(data):
    global p_f
    p_f = [0,0,0]
    p_f[0] = data.point.x
    p_f[1] = data.point.y
    p_f[2] = data.point.z

# def callback_vicon(data):
#     global xpos, ypos,zpos,vx, vy, vz,yaw_v
#     xpos = data.pose.pose.position.x
#     ypos = data.pose.pose.position.y
#     zpos = data.pose.pose.position.z
#     qx = data.pose.pose.orientation.x
#     qy = data.pose.pose.orientation.y
#     qz = data.pose.pose.orientation.z
#     qw = data.pose.pose.orientation.w
#     vx = data.twist.twist.linear.x
#     vy = data.twist.twist.linear.y
#     vz = data.twist.twist.linear.z
#     [roll_v, pitch_v, yaw_v] = quaternion_to_euler(qx,qy,qz,qw)
    #print([qx, qy, qz, qw])
    #print(yaw_v)

def callback_lio(data):
    global xpos, ypos,zpos,vx, vy, vz,yaw_v
    xpos = data.pose.pose.position.x
    ypos = data.pose.pose.position.y
    zpos = data.pose.pose.position.z
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z
    [roll_v, pitch_v, yaw_v] = quaternion_to_euler(qx,qy,qz,qw)
    #print([qx, qy, qz, qw])
    #print(yaw_v)

def callback_imu(imu_data):
    global roll,pitch,yaw, yawrate
    qx = imu_data.orientation.x
    qy = imu_data.orientation.y
    qz = imu_data.orientation.z
    qw = imu_data.orientation.w
    [roll, pitch, yaw] = quaternion_to_euler(qx,qy,qz,qw)
    pitch = pitch
    #print(yaw)
    yawrate = imu_data.angular_velocity.z


def callback_safety(data):
    global land_flag
    land_flag = 1

def callback_start(data):
    global xref, start_flag, yaw_ref, yaw_v
    if start_flag == 0:
        xref[0] = xpos
        xref[1] = ypos
        xref[2] = zpos + 0.5
        yaw_ref = yaw_v
        print(yaw_v)
    start_flag = 1


def callback_ref(data):
    global xref, heading
    xref[0] = data.pose.position.x
    xref[1] = data.pose.position.y
    xref[2] = data.pose.position.z
    heading = data.pose.orientation.z

def trajectory(x, y, z):
    global k
    #print(xref)
    if abs(x - xref[0]) < 0.4 and abs(y - xref[1]) < 0.4 and abs(z - xref[2]) < 0.4 and k < len(traj)-1:
        k = k + 1
        #xref[0] = 1.0 * math.cos(2 * math.pi * k/100)
        #xref[1] = 1.0 * math.sin(2 * math.pi * k/100)
        #xref[2] = 1.0
        xref[0] = round(traj[k][0],5)
        xref[1] = round(traj[k][1],5)
        xref[2] = round(traj[k][2],5)

    # if abs(x - xref[0]) < 0.4 and abs(y - xref[1]) < 0.4 and abs(z - xref[2]) < 0.4:
    #     k = k + 1
    #     xref[0] = 5.0 * math.cos(2 * math.pi * k/100)
    #     xref[1] = 5.0 * math.sin(2 * math.pi * k/100)
    #     xref[2] = xref[2] + 0.04


def callback_sonar(data):
    global zpos
    zpos = data.range * (math.cos(roll) * math.cos(pitch))

def callback_traj(data):
    global xobs1_g
    xobs1_g = data.solution

def callback_traj_2(data):
    global xobs2_g
    xobs2_g = data.solution


def PANOC():
    rospy.init_node('PANOC', anonymous=True)
    pub = rospy.Publisher('command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=1)
    pub_traj = rospy.Publisher('/hummingbird1/traj_1', OptimizationResult, queue_size=1)
    #sub_sonar = rospy.Subscriber('/mavros/distance_sensor/lidarlite_pub', Range, callback_sonar)
    #sub = rospy.Subscriber('/odometry/imu', Odometry, callback_lio)
    #pub_ref = rospy.Publisher('pixyy/reference', PoseStamped, queue_size=1)
    sub = rospy.Subscriber('ground_truth/odometry', Odometry, callback_lio)
    sub_safety = rospy.Subscriber('safety_land', String, callback_safety)
    sub_start = rospy.Subscriber('set_start', String, callback_start)
    sub_imu = rospy.Subscriber('imu', Imu, callback_imu)
    sub_pot = rospy.Subscriber('potential_delta_p_hummingbird', PointStamped, callback_pot)
    sub_traj = rospy.Subscriber('/hummingbird2/traj_2', OptimizationResult, callback_traj)
    sub_traj_2 = rospy.Subscriber('/hummingbird3/traj_3', OptimizationResult, callback_traj_2)
    #sub_ref = rospy.Subscriber('/hummingbird/reference', PoseStamped, callback_ref)
    pub_ref = rospy.Publisher('ref', PoseStamped, queue_size=1)
    rate = rospy.Rate(20) # 20hz
    uold = [9.81, 0.0, 0.0]

    ustar = [9.81,0.0,0.0] * (N)
    i = 0
    t = 0
    safety_counter = 0
    global integrator
    integrator = 0
    global xref, yaw_ref
    xref = [0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0]
    yaw_ref = 0


    ##ADAPT WEIGHT PARAMS####
    Qx_min = [1.5,1.5]
    Qx_max = [5,5]
    Qx_adapt = [0,0]


    xpos_ref = 0
    ypos_ref = 0
    zpos_ref = 1.0


    while not rospy.is_shutdown():
        global p_f, land_flag, start_flag, d1, d2, d, heading
        #p_f = [0,0,0]
        r_s = [0.4,0.4]
        start = time.time()

        trajectory(xpos, ypos, zpos)


        ######BODY ROTATIONS####
        zpos_angle = zpos * (math.cos(roll) * math.cos(pitch))
        x0_body = [math.cos(yaw_v)*xpos + math.sin(yaw_v)*ypos, -math.sin(yaw_v)*xpos + math.cos(yaw_v)*ypos, zpos, vx, vy, vz, roll, pitch]
        if (t < 100) | (land_flag == 1):
            p_f = [0,0,0]

        f_nmhe = [0.0, 0, 0]

        ###ADAPT NMPC WEIGHTS###
        scale = adapt_weights(p_f)
        Qx_adapt[0] = Qx_min[0] + Qx_max[0]*scale
        Qx_adapt[1] = Qx_min[1] + Qx_max[1]*scale

        xref_body = [(math.cos(yaw_v)*xref[0] + math.sin(yaw_v)*xref[1]) + p_f[0], (-math.sin(yaw_v)*xref[0] + math.cos(yaw_v)*xref[1]) + p_f[1], xref[2] + p_f[2], xref[3], xref[4], xref[5], xref[6], xref[7]]
        p_ref_dist = math.sqrt((xref_body[0] - x0_body[0])**2 + (xref_body[1] - x0_body[1])**2 + (xref_body[2] - x0_body[2])**2)
        if p_ref_dist > 1:
            p_ref_norm = [(xref_body[0] - x0_body[0]) / p_ref_dist, (xref_body[1] - x0_body[1]) / p_ref_dist, (xref_body[2] - x0_body[2]) / p_ref_dist]
            xref_body[0:3] = [x0_body[0] + p_ref_norm[0], x0_body[1] + p_ref_norm[1], x0_body[2] + p_ref_norm[2]]

        xobs1_b = global2body(xobs1_g, yaw_v)
        xobs2_b = global2body(xobs2_g, yaw_v)
        # print(xref)

        z0 = x0_body + xref_body + uref + uold + f_nmhe + Qx_adapt+ list(xobs1_b) + list(xobs2_b) + list(r_s)
        solution = mng.call(z0, initial_guess=[9.81,0,0]*(N),buffer_len = 10*4096)
        ustar = solution['solution']
        uold = ustar[0:3]
        own_traj_b = predict(x0_body, ustar)
        #print(own_traj_b)
        own_traj_g = body2global(own_traj_b, yaw_v)
        #print(own_traj_g)

        pubtraj = OptimizationResult()
        pubtraj.solution = own_traj_g
        #print(pubtraj)

        pub_traj.publish(pubtraj)

        u_r = ustar[1]
        u_p = ustar[2]

        rpyt = RollPitchYawrateThrust()
        xref_pub = PoseStamped()

        if land_flag == 0:
            integrator = integrator + 0.001*(xref_body[2] - zpos)

        t0_ = t0 + integrator


        C = 9.81 / t0_
        u_t = ustar[0] / C


        if (t < 40) & (start_flag == 1):
            u_t = 0.2
            u_r = 0
            u_p = 0
            integrator = 0

        if start_flag == 0:
            u_t = 0
            t = 0
            u_r = 0
            u_p = 0
            integrator = 0

        if land_flag == 1:
            u_t = (t0_ - 0.02) - safety_counter*0.00015
            safety_counter+=1
            if (u_t < 0) | (zpos < 0.2):
                u_t = 0
                mng.kill()

        rpyt.roll = u_r
        rpyt.pitch = u_p

        rpyt.thrust.x = 0
        rpyt.thrust.y = 0

        d1 = xref[0] * xref[0] + xref[1] * xref[1]
        d2 = xpos * xpos + ypos * ypos
        d = math.sqrt(abs(d2 - d1))

        heading = math.atan2((xref[1]-ypos),(xref[0]-xpos))
        ang_diff = heading - yaw_v

        rpyt.thrust.z = u_t

        # print(heading)

        # print(yaw_v)

        if abs(ang_diff) > math.pi:
            ang_diff -= math.pi*2
        rpyt.yaw_rate = 0

        if abs(d) < 0.6:
            rpyt.yaw_rate = 0
            
        u_y = 0.7*(ang_diff) - 0.1*yawrate #+ yaw_integrator
        # u_y = 0.7*(ang_diff) - 0.1*yawrate
        if u_y > 1:
            u_y = 1

        if u_y < -1:
            u_y = -1
        
        rpyt.yaw_rate = u_y

        # print(ang_diff)
        # print(rpyt.yaw_rate)

        rpyt.header = std_msgs.msg.Header()
        rpyt.header.stamp = rospy.Time.now()
        rpyt.header.frame_id = 'world'

        pub.publish(rpyt)

        # header = std_msgs.msg.Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = 'odom lidar'

        xref_pub.pose.position.x = xref[0]
        xref_pub.pose.position.y = xref[1]
        xref_pub.pose.position.z = xref[2]

        xref_pub.header.stamp = rospy.Time.now()
        xref_pub.header.frame_id = 'world'

        pub_ref.publish(xref_pub)


        end = time.time()
        #print(end-start)
        rate.sleep()
        #end = time.time()

        t = t + 1


if __name__ == '__main__':
     #rospy.sleep(10)
     try:
         PANOC()
     except rospy.ROSInterruptException:
         pass
