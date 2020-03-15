'''
==================================================================
Space frame generate file
Req. python,numpy
2020.03
==================================================================
'''

'''
====================================================================
Import Part
====================================================================
'''
from FEM_frame import *
import numpy as np
import os
import random

import shutil
import csv
import ast

from os import listdir
from os.path import isfile, join


'''
====================================================================
Class Part
====================================================================
'''

class gen_model:
    def __init__(self,
                num_x
                ,num_z
                ,span
                ,diameter
                ,loadx
                ,loady
                ,loadz
                ,Young,poisson
                ,c1,c2,c3,c4,c5,c6,c7):

        # Frame Dimensions
        self.num_x = num_x
        self.num_z = num_z
        self.span = span
        self.dia = diameter
        self.area = np.pi*(self.dia**2)/4
        self.ival = np.pi*((self.dia)**4)/64
        self.jval = np.pi*((self.dia)**4)/32


        self.span = span

        # Frame Load
        self.loadx = loadx
        self.loady = loady
        self.loadz = loadz

        # Frame Properties
        self.poisson = poisson
        self.Young = Young
        self.shearmodulus = self.Young/2*(1+self.poisson)

        # Gernate lists and number
        self.n_u_x = []
        self.n_u_z = []
        self.n_u_coord = []
        self.n_u_name_div =[]

        # -------------------------------------------------
        # Parameter function
        self.c1 = c1
        self.c2 = c2
        self.c3 = c3
        self.c4 = c4
        self.c5 = c5
        self.c6 = c6
        self.c7 = c7
        # -------------------------------------------------
        # Frame Model
        self.model = None # will be generated after init

        # Generate
        self.gennode()
        self.generate()

    def _Y_Val_Range(self,x,z):
        return ((self.c1*(x**2) + self.c2*(x*z) + self.c3*(z**2) + self.c4*x + self.c5*z + self.c6 ) * self.c7)

    def gennode(self):
        self.n_u_x = []
        self.n_u_z = []
        self.n_u_coord = []

        for i in range(-int(self.num_x/2),int(self.num_x/2)):
            self.n_u_x.append(i*self.span)

        for i in range(-int(self.num_z/2),int(self.num_z/2)):
            self.n_u_z.append(i*self.span)

        for i in range(len(self.n_u_x)):
            for j in range(len(self.n_u_z)):
                self.n_u_coord.append([self.n_u_x[i],self.span*self._Y_Val_Range(self.n_u_x[i],self.n_u_z[j]),self.n_u_z[j]])

    def savetxt(self,name):
        # ------------------------------
        # Write and save output model file
        # ------------------------------

        new_file = open(name, "w+")
        for num1 in range(len(self.model.loads)):
            new_file.write(" {}\r\n".format(self.model.loads[num1]))
        for num1 in range(len(self.model.nodes)):
            new_file.write(" {}\r\n".format(self.model.nodes[num1]))
        for num1 in range(len(self.model.elements)):
            new_file.write(" {},{},{},{},{},{},{},{},{}\r\n".format(
                self.model.elements[num1].name,
                self.model.elements[num1].nodes[0].name,
                self.model.elements[num1].nodes[1].name,
                self.model.elements[num1].em,
                self.model.elements[num1].area,
                self.model.elements[num1].i,
                self.model.elements[num1].sm,
                self.model.elements[num1].j,
                self.model.elements[num1].aor
                ))
        new_file.close()

    def generate(self):
        l1 = Load()
        l1.set_name(1)
        l1.set_type(1)
        l1.set_size(self.loadx,self.loady,self.loadz,self.loadx,self.loady,self.loadz)
        '''
        ==================================
        Generate Node
        ==================================
        '''
        n = 'n'
        #Generate Node upper cord
        n_u_name=[]
        counter = 1
        for i in range(len(self.n_u_coord)):
            n_u_name.append(n+str(counter))
            n_u_name[-1] = Node()
            n_u_name[-1].set_name(counter)
            n_u_name[-1].set_coord(self.n_u_coord[i][0],self.n_u_coord[i][1],self.n_u_coord[i][2])
            n_u_name[-1].set_res(0,0,0,0,0,0)
            n_u_name[-1].set_hinge(0)
            counter+=1

        #Divide n_u_name into zrow
        for i in range(len(self.n_u_z)):
            self.n_u_name_div.append([])
        for i in range(len(n_u_name)):
            for j in range(len(self.n_u_z)):
                if n_u_name[i].coord[2] == self.n_u_z[j]:
                    self.n_u_name_div[j].append(n_u_name[i])

        #Surrounding nodes set_res to (1,1,1,1,1,1)
        for i in range(len(self.n_u_name_div)):
            self.n_u_name_div[i][0].set_res(1,1,1,1,1,1)
            self.n_u_name_div[i][0].set_hinge(0)
            self.n_u_name_div[i][-1].set_res(1,1,1,1,1,1)
            self.n_u_name_div[i][-1].set_hinge(0)

        for i in range(len(self.n_u_name_div[0])):
            self.n_u_name_div[0][i].set_res(1,1,1,1,1,1)
            self.n_u_name_div[0][i].set_hinge(0)
            self.n_u_name_div[-1][i].set_res(1,1,1,1,1,1)
            self.n_u_name_div[-1][i].set_hinge(0)

        #Set node name
        node_pool =[]
        counter = 1
        for i in range(len(n_u_name)):
            node_pool.append(n_u_name[i])
            node_pool[-1].set_name(counter)
            counter +=1
            if node_pool[-1].res[0] != 1:
                node_pool[-1].set_load(l1)
            else:
                pass

        '''
        ==================================
        Generate Member
        ==================================
        '''
        e = 'e'
        E_type1_name =[]
        counter = 1
        for num in range(len(self.n_u_name_div)):
            for i in range((len(self.n_u_name_div[0])-1)):
                E_type1_name.append(e+str(counter))
                E_type1_name[-1] = Element()
                E_type1_name[-1].set_name(counter)
                E_type1_name[-1].set_nodes(self.n_u_name_div[num][i],self.n_u_name_div[num][i+1])
                E_type1_name[-1].set_em(self.Young)
                E_type1_name[-1].set_area(self.area)
                E_type1_name[-1].set_i(self.ival,self.ival,self.ival)
                E_type1_name[-1].set_j(self.jval)
                E_type1_name[-1].set_sm(self.shearmodulus)
                E_type1_name[-1].set_aor(0)
                counter+=1

        E_type2_name =[]
        counter = len(E_type1_name)+1
        for num in range(len(self.n_u_name_div)-1):
            for i in range(len(self.n_u_name_div[0])):
                E_type2_name.append(e+str(counter))
                E_type2_name[-1] = Element()
                E_type2_name[-1].set_name(counter)
                E_type2_name[-1].set_nodes(self.n_u_name_div[num][i],self.n_u_name_div[num+1][i])
                E_type2_name[-1].set_em(self.Young)
                E_type2_name[-1].set_area(self.area)
                E_type2_name[-1].set_i(self.ival,self.ival,self.ival)
                E_type2_name[-1].set_j(self.jval)
                E_type2_name[-1].set_sm(self.shearmodulus)
                E_type2_name[-1].set_aor(0)
                counter+=1

        E_type3_name =[]
        counter = len(E_type1_name)+len(E_type2_name)+1
        for num in range(len(self.n_u_name_div)-1):
            for i in range(len(self.n_u_name_div[0])-1):
                E_type3_name.append(e+str(counter))
                E_type3_name[-1] = Element()
                E_type3_name[-1].set_name(counter)
                E_type3_name[-1].set_nodes(self.n_u_name_div[num][i],self.n_u_name_div[num+1][i+1])
                E_type3_name[-1].set_em(self.Young)
                E_type3_name[-1].set_area(self.area)
                E_type3_name[-1].set_i(self.ival,self.ival,self.ival)
                E_type3_name[-1].set_j(self.jval)
                E_type3_name[-1].set_sm(self.shearmodulus)
                E_type3_name[-1].set_aor(0)
                #E_type3_name[-1].set_aor(45)
                counter+=1
        E_type4_name =[]
        counter = len(E_type1_name)+len(E_type2_name)+len(E_type3_name)+1
        for num in range(len(self.n_u_name_div)-1):
            for i in range(len(self.n_u_name_div[0])-1):
                E_type4_name.append(e+str(counter))
                E_type4_name[-1] = Element()
                E_type4_name[-1].set_name(counter)
                E_type4_name[-1].set_nodes(self.n_u_name_div[num][i+1],self.n_u_name_div[num+1][i])
                E_type4_name[-1].set_em(self.Young)
                E_type4_name[-1].set_area(self.area)
                E_type4_name[-1].set_i(self.ival,self.ival,self.ival)
                E_type4_name[-1].set_j(self.jval)
                E_type4_name[-1].set_sm(self.shearmodulus)
                E_type4_name[-1].set_aor(0)
                counter+=1

        '''
        ==================================
        Generate Model
        ==================================
        '''
        self.model = Model()
        # add load
        self.model.add_load(l1)
        # add nodes
        for i in range(len(n_u_name)):
            self.model.add_node(n_u_name[i])
        # add elements
        for i in range(len(E_type1_name)):
            self.model.add_element(E_type1_name[i])
        for i in range(len(E_type2_name)):
            self.model.add_element(E_type2_name[i])

        for i in range(len(E_type3_name)):
            self.model.add_element(E_type3_name[i])
        for i in range(len(E_type4_name)):
            self.model.add_element(E_type4_name[i])

        self.model.gen_all()
        return self.model



