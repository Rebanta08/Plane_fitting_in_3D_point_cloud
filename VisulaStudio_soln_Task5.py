import numpy as np
import open3d as op3d
from open3d import JVisualizer
from sympy.solvers import solve
from sympy import Symbol
import random
import math
pcd_read=op3d.io.read_point_cloud("record_00348.pcd")                                             
print(pcd_read)                                                                                 
print(np.asarray(pcd_read.points))                                                              
pcd_opt=np.asarray(pcd_read.points) 
_threshold_=float(input("Threshold:"))                                        
_prob_=float(input("Success Prob:"))  
counter=0
equation=0
Num=271983
Num_store=Num
plane=0                                    
while Num_store>counter:
    i= random.randint(0, Num-1)                                                            
    j = random.randint(0, Num-1)
    k = random.randint(0, Num-1)
    p_1 = np.array(pcd_opt[i])                                                                   
    p_2 = np.array(pcd_opt[j])
    p_3 = np.array(pcd_opt[k])
    if i==j or j==k or i==j:                                                              
        i = random.randint(0, Num-1)
        j = random.randint(0, Num-1)
        k = random.randint(0, Num-1)
        p_1 = np.array(pcd_opt[i])
        p_2 = np.array(pcd_opt[j])
        p_3 = np.array(pcd_opt[k])
    v_1 = p_3 - p_1                                                                            
    v_2 = p_2 - p_1
    crossproduct = np.cross(v_1, v_2)                                                                  
    var_1, var_2, var_3 = crossproduct
    dotproduct = np.dot(crossproduct, p_3)
    x = Symbol('x')
    y = Symbol('y')
    z = Symbol('z')
    Out_lier=0
    In_lier=0
    print('Equation: {0}x + {1}y + {2}z = {3}'.format(var_1, var_2, var_3, dotproduct))
    plane=plane+1                                                                        
    for m in range(1,Num):
        equation=var_1*pcd_opt[m,0] + var_2*pcd_opt[m,1] + var_3*pcd_opt[m,2] - dotproduct
        _lier=abs(equation)/math.sqrt(math.pow(var_1,2)+math.pow(var_2,2)+math.pow(var_3,2))                   
        if _lier<=_threshold_:
            In_lier = In_lier+1                                                               
        else:
            Out_lier = Out_lier+1
    eps=Out_lier/(Out_lier+In_lier)                                                        
    Num_store=np.log(1-_prob_)/np.log(1-math.pow(1-eps,3))                                         
    counter=counter+1                                                                          
print(In_lier,"Inliers",Out_lier,"Outliers")
print(plane, "different planes checked")
points = [pcd_opt[i], pcd_opt[j], pcd_opt[k]]                                                          
lin_set = [[0, 1], [0, 2], [1, 2]]
col_set = [[1, 0, 0] for i in range(len(lin_set))]
line_set = op3d.geometry.LineSet()
line_set.points = op3d.utility.Vector3dVector(points)
line_set.lines = op3d.utility.Vector2iVector(lin_set)
line_set.colors = op3d.utility.Vector3dVector(col_set)
op3d.visualization.draw_geometries([line_set, pcd_read])                                         