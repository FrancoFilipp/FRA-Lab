import rosbag
import numpy as np

bag_w = rosbag.Bag('IR_sensor/blanco_5cm_ldr_2024-04-17-10-58-32.bag')
values_list_w = []
for topic, msg, t in bag_w.read_messages():
    values_list_w.append(int(msg.adc0))

bag_w.close()

bag_b = rosbag.Bag('IR_sensor/negro_5cm_ldr_2024-04-17-10-55-07.bag')
values_list_b = []
for topic, msg, t in bag_b.read_messages():
    values_list_b.append(int(msg.adc0))

bag_b.close()

print("WHITE MAX")
print(max(values_list_w))

print("BLACK MAX")
print(max(values_list_b))

print("WHITE MIN")
print(max(values_list_w))

print("BLACK MIN")
print(max(values_list_b))

# descarto el valor maximo y minimo para calcular la media
values_list_w.remove(max(values_list_w))
values_list_b.remove(max(values_list_b))

print("WHITE MEAN 5cm")
mean_w = sum(values_list_w) / len(values_list_w)
mean_b = sum(values_list_b) / len(values_list_b)
print(round(mean_w, 2))

print("BLACK MEAN 5cm")
print(round(mean_b, 2))


print("WHITE STD")
print(round(np.std(values_list_w),2))

print("BLACK STD")
print(round(np.std(values_list_b),2))
