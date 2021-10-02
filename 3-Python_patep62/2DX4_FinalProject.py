import math
import matplotlib.pyplot as plt
import serial

#Parthav Patel
#400251342
#patep62
#2DX4 Final Project
#Python Version 3.9

#Libraries needed: Math, matplotlib, serial.

s = serial.Serial("COM3", 115200)

print("Opening: " + s.name)

z = []
y = []
o = 0 #Variable representing theta of the motor.

#This first for loop will read and print the ToF boot up lines.

for k in range(10):
    g = s.readline()
    c = g.decode()
    print(c)

#The outer for loop is determined by the number of measurements you wish to take.

for i in range(10):

    o = 0 #Reset theta after each 360 spin

    for k in range(128): #For loop for each spin of the motor. Because we are taking 128 measurements per spin, we run the loop 128 times.

        g = s.readline()
        c = int(g.decode())

        z.append(c*math.sin(math.radians(o))) #Convert integer distane measurement to z and y rectangular coordinates.
        y.append(c*math.cos(math.radians(o)))

        print(c) #Print distance for debugging purposes.
        o = o + 2.8125 #Increment angle


print("Closing: " + s.name)
s.close()

x = []

#These for loops will initialize the x list of displacement values.
for a in range(128):
    x.append(0)

for a in range(128):
    x.append(100)

for a in range(128):
    x.append(200)

for a in range(128):
    x.append(300)

for a in range(128):
    x.append(400)

for a in range(128):
    x.append(500)

for a in range(128):
    x.append(600)

for a in range(128):
    x.append(700)

for a in range(128):
    x.append(800)

for a in range(128):
    x.append(900)

#Now plot the 3 lists.
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5);
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()