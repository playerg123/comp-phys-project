# -*- coding: utf-8 -*-
"""
Created on Sat Nov 18 12:33:21 2017

@author: playe
"""
import numpy as np
import matplotlib.pyplot as plt
import itertools as itool
from scipy.spatial import Delaunay

#classes to define starting conditions
class Bfield_config(object):
    uniform,frc,tokamak = range(3)

class profile_config(object):
    random,maxwellian = range(2)
    
class BC_config(object):
    closed,constant_flux,periodic_flux = range(3)

#plasma mesh solver class
class plasma_mesh(object):

    def __init__(self,Lx=0.,Ly=0.,N_init=0,config=0,q_0=0.):
        self.Lx = Lx
        self.Ly = Ly
        self.N = N_init
        self.Grid = False
        #boundary conditions
        if config == BC_config.closed:
            self.q_bc = 0
        elif config == BC_config.constant_flux:
            self.q_bc = q_0
        else:
            raise Exception('Not done yet')
        
    def create_grid(self,config=0):
        if config == Bfield_config.uniform: #create uniform grid of points, then triangulate
            nrows = int(np.sqrt(self.N)*float(self.Ly)/float(self.Lx))
            ncols = int(np.sqrt(self.N)*float(self.Lx)/float(self.Ly))
            ax = float(self.Lx)/float(ncols)
            ay = float(self.Ly)/float(nrows)
            for row, col in itool.product(range(nrows+1),range(ncols+1)):
                if row == 0 and col == 0:
                    self.nodes = np.array([[0,0]])
                else:
                    self.nodes = np.append(self.nodes,[[ax*col,ay*row]],axis=0)
            self.Grid = Delaunay(self.nodes,incremental=True) #allow incrementing to add points
            self.N = np.shape(self.nodes)[0] #fix N to match actual number of vertices
        else:
            raise Exception('Define a vaild magnetic configuration')
        
            
    def create_cell_values(self,config=0,T_0=0.,n_0=0):
        self.ncells = np.shape(self.Grid.simplices)[0]
        if config == profile_config.random: #random starting T and n profile
            self.T = list(T_0*np.random.random(self.ncells))
            self.n = list(T_0*np.random.random(self.ncells))
        elif config == T_config.maxwellian: #2D maxwellian starting profile w/ randomization
            raise Exception('I\'m not done with this yet')
        else:
            raise Exception('Define a valid temperature initialization')
            
#    def conduction_radiation(self,grad_Tx,grad_Ty,radiation_func):
        
                
    def cell_solver(self):
        q_checker = np.zeros((3,self.ncells))
        for cell in range(self.ncells+1):
            q_nn = []
            for neighbor in range(3):
                if self.Grid.neighbors[cell][neighbor] == -1: #if one of the "neighbors" is the boundary
                    q_nn.append(self.q_bc)
                else:
                    boundary_pts = []
                    for pts in range(3):
                        if pts != neighbor:
                            boundary_pts.append(self.nodes[self.Grid.simplices[cell][pts]])
                    
                        
    
    def update_grid_points(self):
        if not self.Grid:
            raise Exception('Grid must be initialized first, use method "create_grid"')
        self.Grid.add_points([[0.45,0.05],[0.45,0.15]])
        self.nodes = np.append(self.nodes,[[0.45,0.05],[0.45,0.15]],axis=0)
        
A = plasma_mesh(1,1,5,0)
A.create_grid(0)
#A.update_grid_points()
x = A.Grid
plt.triplot(A.nodes[:,0],A.nodes[:,1], x.simplices.copy())
plt.scatter(A.nodes[:,0],A.nodes[:,1])