import matplotlib.pyplot as plt

f1 = open("darknet_ros_hz")
f2 = open("ssd_hz")

hzs1 = []
hzs2 = []
for line in f1:
    hzs1.append(float(line.split()[2]))

for line in f2:
    hzs2.append(float(line.split()[1]))


plt.title("Comparativa FPS\nDarknet ROS vs SSD Inception V2")
plt.plot(list(range(60)), hzs1[:60], 'b', list(range(60)), hzs2[:60], 'r')
plt.xlabel("Tiempo (s)")
plt.ylabel("FPS")
plt.legend(['Darknet ROS', 'SSD Inception V2'])
plt.axis([0, 60, 0, 40])
plt.show()

f1.close()
f2.close()