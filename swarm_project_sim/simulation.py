import math
import matplotlib.pyplot as plt
import random
from class_simulation import Robot
from matplotlib.animation import FuncAnimation
import imageio


def update_plot(frame, cnt):
    plt.clf()
    plt.scatter(0, 0, marker='x')  # Plotting the origin
    
    # Generate the combined list of coordinates for the current frame
    comb = frame
    
    # Plot each point and its label
    for id, r, a in comb:
        x = r * math.cos(a)
        y = r * math.sin(a)
        plt.scatter(x, y, marker='o', color='blue')  # Plotting the polygon vertices
        plt.text(x, y, f"Robot_{id}", ha='right', va='bottom', fontsize=10, color='black')
    
    plt.scatter(anchor_point[0], anchor_point[1], marker='o', color='green')
    plt.title('Polygon Plot using Polar Coordinates')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.xlim(-2, 2)  # Set x-axis limits
    plt.ylim(-2, 2)  # Set y-axis limits
    plt.savefig(f'frame_{cnt}.png')

# Take input for the number of sides
num_coords = int(input("Enter the number of sides for the polygon: "))

# Calculate the interior angle of the polygon
Interior_angle = 360 / num_coords

# Generate polar coordinates for the polygon vertices
agent_coordinates = [[i+1, 1, math.radians(0) + math.radians(Interior_angle) * i] for i in range(num_coords)]

agent_objects = []

for idx, coord in enumerate(agent_coordinates):
    obj = Robot(coord[0], coord[1], coord[2])
    agent_objects.append(obj)    

# Convert polar coordinates to Cartesian coordinates
cartesian_coords = [(r * math.cos(theta), r * math.sin(theta)) for _, r, theta in agent_coordinates]
frames = []
def calculate_interior_angles(coords):
    angles = []
    loops = 0
    for i in range(len(coords)-1):
        _,_, angle1 = coords[i]
        _,_, angle2 = coords[i - 1]
        _,_, angle3 = coords[i + 1]
        angle1 = math.degrees(angle1)
        angle2 = math.degrees(angle2)
        angle3 = math.degrees(angle3)
        print("Current angle: ", angle1)
        print("Previous angle: ", angle2)
        print("Next angle: ", angle3)   
        print(f"angle{i}_12", abs(-angle1+angle2))
        print(f"angle{i}_13", abs(-angle1+angle3))
        loops += 1
    print ("Number of loops: ", loops)

    # return angles

# calculate_interior_angles()

# Calculate Euclidean distance for each Cartesian coordinate
distances = [math.sqrt(x ** 2 + y ** 2) for x, y in cartesian_coords]

# Print the Euclidean distances
for i, distance in enumerate(distances):
    print(f"Distance of point {i + 1} from origin: {distance:.2f}")

# Unzip the Cartesian coordinates into separate lists for x and y
x_coords, y_coords = zip(*cartesian_coords)

# Plot the coordinates
plt.scatter(0, 0, marker='x', color='red')  # Plotting the origin
plt.scatter(x_coords, y_coords, marker='o', color='blue')  # Plotting the polygon vertices
jdx = 1
for j in range (len(x_coords)):
    plt.scatter(x_coords[j], y_coords[j], marker='o', color='blue')  # Plotting the polygon vertices
    plt.text(x_coords[j], y_coords[j], f"Robot_{jdx}", ha='right', va='bottom', fontsize=10, color='black')
    jdx += 1
plt.title('Polygon Plot using Polar Coordinates')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.grid(True)
# max_range = max(max(x_coords), max(y_coords))  # Get the maximum absolute value of coordinates
plt.xlim(-2, 2)  # Set x-axis limits
plt.ylim(-2, 2)  # Set y-axis limits
plt.savefig(f'frame_0.png')
plt.show()


random_point = random.randint(0, num_coords-1)
#random_point=0
how_many=6
combined_list_of_list = []
choice_for_addremove = int(input("Enter choice add:1 / remove:2? "))

if choice_for_addremove == 1:
    pass
elif choice_for_addremove == 2:
    print(random_point)
    for idx, coord in enumerate(agent_coordinates):
        ids, _, a = coord
        print(f"Robot {ids}: ", math.degrees(a))
    for i in range(how_many):
        pop_index=random_point
        if pop_index>len(agent_coordinates)-1 or random_point==0:
            print("Im here")
            pop_index=0
        agent_coordinates.pop(pop_index)
        cartesian_coords.pop(pop_index)
    print("Before slicing: ",agent_coordinates)
    # if random_point == len(agent_coordinates) - 2:
    #     random_point = -1
    # calculate_interior_angles(agent_coordinates)
    if random_point - 1 >= len(agent_coordinates):
        print("I'm here 2nd")
        random_point = len(agent_coordinates)
    first_slice=agent_coordinates[random_point-1:]
    print(first_slice)
    second_slice=agent_coordinates[:random_point-1]
    print(second_slice)
    combined_list=first_slice+second_slice
    angle_right=abs(combined_list[0][2]-combined_list[1][2])
    angle_left=abs(combined_list[-1][2]-combined_list[0][2])
    if angle_left>math.radians(180):
        angle_left=math.radians(360)-angle_left
    if angle_right>math.radians(180):
        angle_right=math.radians(360)-angle_right    
    print("Before-------")    
    for idx, coord in enumerate(combined_list):
        ids, _, a = coord
        print(f"Robot {ids}: ", math.degrees(a))
    print("angle left: ", math.degrees(angle_left))
    print("angle right: ", math.degrees(angle_right))
    print("After------")
    # anchor_index = random_point - 1
    # anchor_point = cartesian_coords[anchor_index]
    # anchor_id = agent_coordinates[anchor_index][0]
    anchor_point = [combined_list[0][1]*math.cos(combined_list[0][2]), combined_list[0][1]*math.sin(combined_list[0][2])]
    count = 1
    n=(360/math.degrees(angle_left))
    
    while(round(math.degrees(angle_left))!=round(math.degrees(angle_right))):
        update_plot(combined_list, count)
        print("Iteration ",count)
        
        # print("Existing number of sides: ", n)
        new_inter=math.radians(360/(n-count))
        # print("New internal angle: ", new_inter)
        for j in range(1, len(combined_list)):
            if combined_list[j-1][-1]+new_inter > math.radians(360):
                combined_list[j][-1]=combined_list[j-1][-1]+new_inter - math.radians(360) 
            else:
                combined_list[j][-1]=combined_list[j-1][-1]+new_inter
        for idx, coord in enumerate(combined_list):
            ids, _, a = coord
            print(f"Robot {ids}: ", math.degrees(a))
        
        

        angle_right=abs(combined_list[0][2]-combined_list[1][2])
        angle_left=abs(combined_list[-1][2]-combined_list[0][2])
        if angle_left>math.radians(180):
            angle_left=math.radians(360)-angle_left
        if angle_right>math.radians(180):
            angle_right=math.radians(360)-angle_right
        count+=1
        
        print("angle left: ", angle_left)
        print("angle right: ", angle_right)
        combined_list_of_list.append(combined_list)
    
    update_plot(combined_list, count)
    images = []
    for i in range(0, count+1):
        images.append(imageio.imread(f'frame_{i}.png'))
    imageio.mimsave('animation.gif', images, fps=0.65, loop=0)
    

        
    # print("Interior angles after removal:", [math.degrees(angle) for angle in interior_angles])
    
    # Unzip the Cartesian coordinates into separate lists for x and y
    # cartesian_coords.pop(random_point)

    
    # Plot the coordinates
    

    







