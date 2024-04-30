

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.


# Written by Om Patel, patelo7, 400378073
import serial
import io
import numpy as np
import open3d as o3d
from time import sleep
s = serial.Serial('COM4', 115200, timeout = 1)
f = open('tof_radar.xyz','w')  # w : writing mode  /  r : reading mode  /  a  :  appending mode
                            
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()
z_count = 0;
# wait for user's signal to start the program

# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

a = input("Press Enter to start communication...")
s.write('s'.encode())
for i in range(4):
    x = s.readline()                    # read one line
    print(x.decode('utf-8'))

while(1):
    # recieve characters from UART of MCU
    i = 0
    for i in range(32):
        skip = 0
        x = s.readline()
        while (x.decode('utf-8') == ''):
            x = s.readline()
        y = s.readline()

        x1 = x.decode('utf-8')
        y1 = y.decode('utf-8')
        if (x1 == 't\r' or y1 == 't\r'):
            break
        if (x1 == 'sk\r' or y1 == 'sk\r'):
            skip = 1
            for j in range(i,32):
                f.write('0' + '\t' + '0' + '\t' + str(z_count) +'\n')
                print('0', '0', z_count)
                
        x2 = "\n".join(x1.split())
        y2 = "\n".join(y1.split())

        if (skip == 0):
            f.write(x2 + '\t' + y2 + '\t' + str(z_count) +'\n')    #write x,y,z to file as p1
            print(x2, y2, z_count)
        elif (skip == 1):
            break

    if (x1 == 't\r' or y1 == 't\r'):
        break
    
    z_count = z_count + 100 # corresponding to 10cm (100 mm) displacement

#close the port
print("\nClosing: " + s.name)
s.close()
f.close()

print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("tof_radar.xyz", format='xyz')

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))
size, col = (np.asarray(pcd.points)).shape
#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")

#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0,size):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice        
lines = []
num_of_slices = int(np.asarray(pcd.points)[-1][2]/100 + 1) # +1 to account for z = 0
perslice = int(size/num_of_slices) #should be 32 with 11.25 deg config
for x in range(0,num_of_slices):
    for y in range(0, perslice):
        if (y == perslice-1):
            lines.append([yz_slice_vertex[y+(x*perslice)], yz_slice_vertex[x*perslice]])
        else:
            lines.append([yz_slice_vertex[y+(x*perslice)], yz_slice_vertex[y+(x*perslice)+1]])


##the following is used for "2dx4data.xyz", kinda broken tho
#Define coordinates to connect lines between current and next yz slice        
for x in range(0,num_of_slices):
    for y in range(0,perslice):
        if (x == num_of_slices-1):
            break
        else:
            lines.append([yz_slice_vertex[y+(x*perslice)], yz_slice_vertex[y+((x+1)*perslice)]])

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
