'''
==================================================================
Eample file for Space frame analysis and export
Req. python,numpy
2020.03
==================================================================
'''

'''
====================================================================
Import Part
====================================================================
'''
from frame_GEN import *

'''
====================================================================
Parameter Part
====================================================================
'''
num_x = 20  # amount of x span (horizontal)
num_z = 20  # amount of z span (horizontal)
span  = 1   # span length (m.)
diameter = 0.05   # member's diameter  (m.)
loadx = 0   # load in x direction (N)
loady = -int((np.random.randint(2,6))*0.5)*1000 # load in y direction (N)
loadz = 0   # load in z direction (N)
Young = 10000000000   # member's Young modulus (N/sqm)
poisson = 0.15   # member's Poisson ration

'''
====================================================================
Form Parameter Part
y (heigth) of each node will be represented by
y = (c1*(x**2) + c2*(x*z) + c3*(z**2) + c4*x + c5*z + c6) * c7
or please feel free to hack at
frame_GEN
class gen_model's def _Y_Val_Range
====================================================================
'''
c1= 2
c2= -0.2
c3= 2
c4= 0
c5= 0
c6= 0.3
c7= -0.05



'''
====================================================================
Generate model
====================================================================
'''
model_X = gen_model(num_x,num_z,span,diameter,loadx,loady,loadz,Young,poisson,c1,c2,c3,c4,c5,c6,c7)

'''
====================================================================
Print out global deformations of each node
====================================================================
'''
for i in range(len(model_X.model.nodes)):
    print(model_X.model.nodes[i].global_d)

'''
====================================================================
Print out Strain energy of the model
====================================================================
'''
print(model_X.model.U_full[0][0])

'''
====================================================================
Export as .obj file
====================================================================
'''
name = 'Frame.obj'
model_X.model.gen_obj(name)
