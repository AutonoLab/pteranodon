import math
import numpy as np
import xml.etree.ElementTree as ET
import json

LOG_FOLDER = "/home/jmcgowen/Documents/2021-summer-Justin/GazeboLogs/echo_logs_fast/"
#       <robotNamespace/>
#       <jointName>rotor_0_joint</jointName>
#       <linkName>rotor_0</linkName>
#       <turningDirection>ccw</turningDirection>
#       <timeConstantUp>0.0125</timeConstantUp>
#       <timeConstantDown>0.025</timeConstantDown>
#       <maxRotVelocity>1100</maxRotVelocity>
#       <motorConstant>5.84e-06</motorConstant>
#       <momentConstant>0.06</momentConstant>
#       <commandSubTopic>/gazebo/command/motor_speed</commandSubTopic>
#       <motorNumber>0</motorNumber>
#       <rotorDragCoefficient>0.000175</rotorDragCoefficient>
#       <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
#       <motorSpeedPubTopic>/motor_speed/0</motorSpeedPubTopic>
#       <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>

def get_power(torque, speed):
    # https://www.faulhaber.com/en/support/technical-support/motors/tutorials/dc-motor-tutorial-dc-motor-calculation/
    # https://github.com/PX4/PX4-SITL_gazebo/issues/110

    #Typical batteries seems to output 610.5 watts continous?

    #Turns out none of this matters without friction. Welp.
    #return (abs(torque) * abs(speed))

    # Thrust at steady flight: .0383 (from 5.84e-06*(speed**2))
    # Total weight: 15.043 newtons
    # Need (15.043/4) / .0383 = 98.1919 * more force
    # Just say speed is sqrt(98.192) = 9.909 times higher? Nope, still too low... somehow both torque and speed are wrong I guess.

    #Files define a slowdown of 10x for simulation purposes. Not sure how this effects torque... just rerun?

    torque = abs(torque)
    speed = abs(speed)
    return torque * speed
    # Moment Constant = 60 / (2π * Kv)
    KV_FROM_KM = False
    if KV_FROM_KM:
        # From gazebo definitions
        kM = 0.06
        # This works out to about 160
        kV = 60.0 / (2.0 * math.pi * kM)
    else:
        # kV = 850.0 #? http://www.arducopter.co.uk/iris-quadcopter-uav.html
        #      or 880 from the specs of the actual drone the lab has?
        kV = 850
        kM = 60.0 / (2.0*math.pi*kV)
    # speed will already be in radians
    rpm = speed * 60.0 / (2 * math.pi)

    V = rpm /kV

    # V = kV/rpm?
    # V = (torque / kM) * R + kE * speed

    ki = 1.0 / kM

    # torque here should technically have friction added, no load current ~= .6 A
    # Wikipedia say kM = torque/sqrt(P), going with the thing from the PX4 forum
    Iload = torque * ki



    Pin = Iload * V
    print(Pin)
    print(abs(torque * speed)/Pin)
    print("THRUST: ", 5.84e-06*(speed**2))
    #return torque
    #return rpm
    #return ( 5.84e-06*(speed**2))
    return Pin

#The states are logged at 15000 states/min, drones tend to cap at about 6000 it seems, so this shooould work.
#I can also just decrease the timestep?
def rpy_to_speed(time1, time2, roll1, roll2):
    roll1 = roll1 % (2.0 * np.pi)
    roll2 = roll2 % (2.0 * np.pi)

    diff = abs(roll1 - roll2)
    diff = min(diff, 2.0 * np.pi - diff)

    return diff / (time2 - time1)


# Torque seems rather noisy, hopefully this works (or at least with a rolling average), spikes between normal values and 1E-11 a lot?
def step_through_torques_and_rolls_old(sample_times, torques, torque_times, rolls, roll_times):
    i = 0
    j = 0

    powers = []

    for time in sample_times:
        while torque_times[i] <= time:
            i += 1
        while roll_times[j] <= time:
            j += 1
        # lirp torque
        a = (time - torque_times[i - 1]) / (torque_times[i] - torque_times[i - 1])
        torque = a * torques[i] + (1.0 - a) * torques[i - 1]
        # don't lirp, need time for speed already
        speed = rpy_to_speed(roll_times[i - 1], roll_times[i], rolls[i - 1], rolls[i])
        #print(torque, speed)
        powers.append(get_power(torque, speed))
    return powers

def lirp(time, i, times, vals):
    alp = (time - times[i-1]) / (times[i] - times[i-1])
    return alp * vals[i] + (1.0 - alp) * vals[i-1]

def step_through_torques_and_angvels(sample_times, torques, torque_times, angvels, angvel_times):
    i = 0
    j = 0

    powers = []

    for time in sample_times:
        while torque_times[i] <= time:
            i += 1
        while angvel_times[j] <= time:
            j += 1
        # lirp, sampled at ?100hz?
        torque = lirp(time, i, torque_times, torques)
        speed = lirp(time, j, angvel_times, angvels)
        #print(torque, speed)
        #torque = (angvels[j] - angvels[j-1])/(angvel_times[j]-angvel_times[j-1]) * 9.75e-07

        powers.append(get_power(torque, speed))
    return powers

    i = 0
    j = 0

    powers = []

    for time in sample_times:
        while torque_times[i] <= time:
            i += 1
        while angvel_times[j] <= time:
            j += 1
        # lirp, sampled at ?100hz?
        torque = lirp(time, i, torque_times, torques)
        speed = lirp(time, j, angvel_times, angvels)
        #print(torque, speed)
        #torque = (angvels[j] - angvels[j-1])/(angvel_times[j]-angvel_times[j-1]) * 9.75e-07

        powers.append(get_power(torque, speed))
    return powers

def torque_log_extractor(filename):
    # Unfortunately, the gazebo output is not in json. Just directly grab the corresponding lines...
    times = []
    torques = []
    time = 0.0
    z_is_torque = False
    with open(filename, 'r') as file1:
        Lines = file1.readlines()

        for line in Lines:
            line = line.strip()
            # Add secs and nanosecs
            if line.startswith("sec"):
                time = float(line.split()[1])
            elif line.startswith("nsec"):
                time += 1e-9 * float(line.split()[1])
                times.append(time)
            elif line.startswith("z"):
                # z's alternate between forces and torques. Torque is only in z (for motor, blades can still bend I guess?)
                if z_is_torque:
                    torques.append(float(line.split()[1]))
                z_is_torque = not z_is_torque
    return (times, torques)

def angvel_log_extractor(filename):
    # Unfortunately, the gazebo output is not in json. Just directly grab the corresponding lines...
    times = []
    angvels = []
    time = 0.0
    z_is_angvel = 1 #only angvel if 0
    with open(filename, 'r') as file1:
        Lines = file1.readlines()

        for line in Lines:
            line = line.strip()
            # Add secs and nanosecs
            if line.startswith("sec"):
                time = float(line.split()[1])
            elif line.startswith("nsec"):
                time += 1e-9 * float(line.split()[1])
                times.append(time)
            elif line.startswith("z"):
                # z's alternate between forces and torques. Torque is only in z (for motor, blades can still bend I guess?)
                if z_is_angvel == 0:
                    angvels.append(float(line.split()[1]))
                z_is_angvel -= 1
                if z_is_angvel < 0:
                    z_is_angvel = 2
    return (times, angvels)

def base_vel_log_extractor(filename):
    # Unfortunately, the gazebo output is not in json. Just directly grab the corresponding lines...
    times = []
    latest_vel = []
    vels = []
    time = 0.0
    val_is_vel = 0 #only velocity if < 3
    with open(filename, 'r') as file1:
        Lines = file1.readlines()

        for line in Lines:
            line = line.strip()
            # Add secs and nanosecs
            if line.startswith("sec"):
                time = float(line.split()[1])
            elif line.startswith("nsec"):
                time += 1e-9 * float(line.split()[1])
                times.append(time)
            elif line.startswith("x") or line.startswith("y") or line.startswith("z"):
                # z's alternate between forces and torques. Torque is only in z (for motor, blades can still bend I guess?)
                if val_is_vel < 3:
                    latest_vel.append(float(line.split()[1]))
                if val_is_vel == 2:
                    vels.append(np.array(latest_vel))
                    latest_vel = []
                val_is_vel += 1
                if val_is_vel > 5:
                    val_is_vel = 0
    return (times, vels)

# Deprecated, using IMU results instead
pass
# def roll_log_extractor(filename):
#     # parse xml
#     times = []
#     rotor0 = []
#     rotor1 = []
#     rotor2 = []
#     rotor3 = []
#     tree = ET.parse(filename)
#     root = tree.getroot()
#     for chunk in root.findall("chunk"):
#         for state in chunk.findall("sdf/state"):
#             time_text = state.find("sim_time").text
#             times.append(float(time_text.replace(" ", ".")))
#             for link in state.findall("model/link"):
#                 #last value in link pose is roll
#                 roll = float((link.find("pose").text).split()[-1])
#                 if link.attrib["name"] == "rotor_0":
#                     rotor0.append(roll)
#                 elif link.attrib["name"] == "rotor_1":
#                     rotor1.append(roll)
#                 elif link.attrib["name"] == "rotor_2":
#                     rotor2.append(roll)
#                 elif link.attrib["name"] == "rotor_3":
#                     rotor3.append(roll)
#     return times, rotor0, rotor1, rotor2, rotor3

#(roll_times, roll0, roll1, roll2, roll3) = roll_log_extractor("/home/jmcgowen/.gazebo/test-log-output.xml")
pass
#####################################

(torque_times0, torques0) = torque_log_extractor(LOG_FOLDER+"default-iris-rotor_0_joint-force_torque-wrench.log")
(angvel_times0, angvels0) = angvel_log_extractor(LOG_FOLDER+"default-iris-rotor_0-rotor_0_imu-imu.log")
(torque_times1, torques1) = torque_log_extractor(LOG_FOLDER+"default-iris-rotor_1_joint-force_torque-wrench.log")
(angvel_times1, angvels1) = angvel_log_extractor(LOG_FOLDER+"default-iris-rotor_1-rotor_1_imu-imu.log")
(torque_times2, torques2) = torque_log_extractor(LOG_FOLDER+"default-iris-rotor_2_joint-force_torque-wrench.log")
(angvel_times2, angvels2) = angvel_log_extractor(LOG_FOLDER+"default-iris-rotor_2-rotor_2_imu-imu.log")
(torque_times3, torques3) = torque_log_extractor(LOG_FOLDER+"default-iris-rotor_3_joint-force_torque-wrench.log")
(angvel_times3, angvels3) = angvel_log_extractor(LOG_FOLDER+"default-iris-rotor_3-rotor_3_imu-imu.log")
(vel_times, vels)         = base_vel_log_extractor(LOG_FOLDER+"default-iris::base_link.log")


#Unfortunately, logs are started with slight delays from each other, have to sample still
#Thing angvels3 should always be last, base_vel always first
start_time = max(torque_times0[0], angvel_times0[0], vel_times[0], angvel_times3[0])
end_time = min(vel_times[-1], torque_times0[-1], angvel_times3[-1])


sample_step = .005
sample_times = np.arange(start_time, end_time, sample_step)


vel_mags = [np.linalg.norm(v) for v in vels]

import matplotlib.pyplot as plt
#plt.plot(vel_times, vel_mags)
#plt.show()

powers0 = step_through_torques_and_angvels(sample_times, torques0, torque_times0, angvels0, angvel_times0)
powers1 = step_through_torques_and_angvels(sample_times, torques1, torque_times1, angvels1, angvel_times1)
powers2 = step_through_torques_and_angvels(sample_times, torques2, torque_times2, angvels2, angvel_times2)
powers3 = step_through_torques_and_angvels(sample_times, torques3, torque_times3, angvels3, angvel_times3)

def bad_max_filter(time_range, times, vals):
    new_vals = []
    for i in range(len(vals)):
        j = 0
        new_val = -1
        while times[j] < times[i] - time_range:
            j += 1
        while j < len(times) and times[j] < times[i] + time_range:
            new_val = max(vals[j], new_val)
        new_vals.append(new_val)

import scipy.ndimage
plt.rcParams.update({'font.size': 7})

summed_powers = [powers0[i] + powers1[i] + powers2[i] + powers3[i] for i in range(len(powers0))]

#Can't really justify .016
fancy_powers = scipy.ndimage.maximum_filter(summed_powers, size=math.floor(0.0526132/sample_step)*2)

#plt.plot(sample_times, fancy_powers)
#plt.show()

power_plot_start = 62.0
power_plot_end   = 62.5

#Completely arbitrary
max_power = 392
#385.8
#4.667
#i_base = sample_times.tolist().index(power_plot_start)
i = 0
while sample_times[i+1] < power_plot_start:
    i += 1
i_base = i
time_range = np.arange(power_plot_start, power_plot_end, .005)

compute_power = [4.667 if fancy_powers[i_base + i] > max_power - 8.488 else 8.488 for i in range(len(time_range)) ]
total_power =   [fancy_powers[i_base + i] + compute_power[i] for i in range(len(time_range))]
max_powers =    [max_power for i in range(len(time_range))]
compute_time =  [ 0.04124712 if fancy_powers[i_base + i] > max_power - 8.488 else 0.00225937 for i in range(len(time_range)) ]

(fig, ax1) = plt.subplots(figsize=(3.5,2.5))

l1 = ax1.plot(time_range, max_powers, label = "Available Power", ls = ":")
l2 = ax1.plot(time_range, compute_power, label = "Compute Power")
l3 = ax1.plot(time_range, fancy_powers[i_base:i_base+len(time_range)], label = "Rotor Power")
l4 = ax1.plot(time_range, total_power, label = "Total Power")
ax1.set_ylim([0, 400])
ax1.set_ylabel('Power (W)')
ax1.set_xlabel('Flight Time (s)')

ax2 = ax1.twinx()
l5 = ax2.plot(time_range, compute_time, label = "Latency", color = "purple", ls = "--")
ax2.set_ylim([-.025, .2])
ax2.set_ylabel('Latency (s)')

lns = l1+l2+l3+l4+l5
labs = [l.get_label() for l in lns]

ax2.legend(lns, labs, loc='best', bbox_to_anchor=(0.00, 0.4, 0.5, 0.5))
plt.title("Latency-Power Tradeoff")

for ax in [ax1, ax2]:
    box = ax.get_position()

    ax.set_position([box.x0, box.y0 + box.height*.05,
                      box.width*.95, box.height * .95])

plt.show()

STEPS_BEFORE_PURSUIT = 30

pursuit_times = [i*1.0/6.0 for i in range(STEPS_BEFORE_PURSUIT*2)]
#COMPLETELY MADE UP DATA:
#(Basically need velocity to vary by 20x for this to work)
adv_vels = [0 for i in range(STEPS_BEFORE_PURSUIT)] + [1 + 10 * ((i)/STEPS_BEFORE_PURSUIT)**2 for i in range(STEPS_BEFORE_PURSUIT)]

#THIS BITS FINE THOUGH:
required_latency = [0.0 for i in range(STEPS_BEFORE_PURSUIT)] + [0.035/adv_vels[i] for i in range(STEPS_BEFORE_PURSUIT, STEPS_BEFORE_PURSUIT*2)]
latencies = [0.00883762 for i in range(STEPS_BEFORE_PURSUIT)] + [.00225937 if required_latency[i] < .00883762 else .00883762 if required_latency[i] < 0.02387 else .02387 for i in range(STEPS_BEFORE_PURSUIT, STEPS_BEFORE_PURSUIT*2) ]

powers = [4.667 for i in range(STEPS_BEFORE_PURSUIT)] +     [8.488103 if required_latency[i] < .00883762 else 4.667 if required_latency[i] < 0.02387 else 10.1167 for i in range(STEPS_BEFORE_PURSUIT, STEPS_BEFORE_PURSUIT*2) ]
accuracies = [55.3 for i in range(STEPS_BEFORE_PURSUIT)] +  [55.3 if required_latency[i] < .00883762 else 55.3 if required_latency[i] < 0.02387 else 57.9 for i in range(STEPS_BEFORE_PURSUIT, STEPS_BEFORE_PURSUIT*2) ]


(fig, axs) = plt.subplots(1, 2, figsize=(3.5,2.5))
plt.suptitle("Search/Pursuit Modes Scenario")
fig.tight_layout()
ax1 = axs[0]
l1 = ax1.plot(pursuit_times, adv_vels, label = "Adeversary Velocity", color = "purple", ls = "--")
#l4 = ax1.plot(pursuit_times, [vel - 1 for vel in adv_vels], label = "Self Velocity", color = "hotpink", ls = ":")
ax1.set_ylim([0, 22])
ax1.set_ylabel('Speed (m/s)')
ax1.set_xlabel('Flight Time (s)')

ax2 = ax1.twinx()
l2 = ax2.plot(pursuit_times, required_latency, label = "Required Latency", ls = ":")
l3 = ax2.plot(pursuit_times, latencies, label = "Compute Latency")
ax2.set_ylim([0, .04])
ax2.set_ylabel('Latency (s)')
ax2.plot([5.0, 5.0000000001], [0, .04], color = "black", ls=":")

lns = l1+l2+l3#+l4
labs = [l.get_label() for l in lns]

ax1.set_title("Adv. Velocity vs Latency")
ax2.legend(lns, labs, loc="upper center", bbox_to_anchor=(0.5, -0.25))

ax3 = axs[1]
l1 = ax3.plot(pursuit_times, powers, label = "Compute Power")
ax3.set_ylim([0, 12])
ax3.set_ylabel('Power (W)')
ax3.plot([5.0, 5.0000000001], [0, 12], color = "black", ls=":")
ax3.set_xlabel('Flight Time (s)')

ax4 = ax3.twinx()
ax4.get_yaxis().set_visible(False)
ax4.plot(pursuit_times, adv_vels, label = "Adeversary Velocity", color = "thistle", ls = "--")
ax4.set_ylim([0, 22])
#ax3.set_ylabel('Speed (m/s)')


ax5 = ax3.twinx()
l2 = ax5.plot(pursuit_times, accuracies, label = "Detection Accuracy", color = "green")
ax5.set_ylim([40, 90])
ax5.set_ylabel('mAP (%)')

lns = l1+l2
labs = [l.get_label() for l in lns]

ax3.set_title("Adv. Velocity vs Acc/Power")

for ax in [ax1, ax2, ax3, ax4]:
    box = ax.get_position()

    x_pos = box.x0
    if ax == ax3 or ax4:
        x_pos = box.x0 + box.width*.2
    ax.set_position([box.x0+box.width*.02, box.y0 + box.height * 0.2,
                      box.width*.8, box.height * 0.8])

ax4.legend(lns, labs, loc="upper center", bbox_to_anchor=(0.5, -0.25))

plt.show()


#power0 = step_through_torques_and_rolls(sample_times, torques0, torque_times0, roll0, roll_times)
#print(power0)
