import numpy as np
import numpy as np

Distances=[]
Angles=[]
is_initialised=False

index=np.where(Distances==0)
Distances=np.delete(Distances, index)
Angles=np.delete(Angles, index)
Distances = np.array(Distances)
Angles = np.array(Angles)
Angles = np.deg2rad(Angles)
x_coord=[]
y_coord=[]


new_distances=[]
new_angles=[]
for d in range(len(Distances)):
      x_coord.append(Distances[d]*np.cos(Angles[d]))
      y_coord.append(Distances[d]*np.sin(Angles[d]))
i=0

points=[]
new_lines=[]
while i<= len(Distances)-1:
    for j in range(0,len(Distances)-2):
      if abs(x_coord[j]*np.cos(Angles[i])+y_coord[j]*np.sin(Angles[i])-Distances[i])<15:
          points=np.append(points,j)
      
    if len(points) >= 40:
        points = points.astype('int32')
        if is_initialised==False:
            new_distances.append(Distances[i])
            new_angles.append(Angles[i])
        new_lines.append([x_coord[i],y_coord[i]])
        Distances=np.delete(Distances, points)
        Angles=np.delete(Angles,points)
        x_coord=np.delete(x_coord,points)
        y_coord=np.delete(y_coord,points)
        points=[]
        i=0
    else:
          i=i+1


if is_initialised==False:
   new_distances=np.array(new_distances)
   new_angles=np.array(new_angles)
   difference_array_90 = abs(new_angles-1.5708)
   indexth1=difference_array_90.argmin()
   x=new_distances[indexth1]
   difference_array_0 = abs(new_angles-0)
   indexth2=difference_array_0.argmin()
   y=2385-new_distances[indexth2]
   theta=((new_angles[indexth1]-1.5708)+(new_angles[indexth2]-0))/2
   new_lines=old_lines
   # return x, y, theta
else:
  new_lines=np.array(new_lines)
  old_lines=np.array(old_lines)
  T=icp(old_lines,new_lines)
  new_lines=old_lines
  # return T
