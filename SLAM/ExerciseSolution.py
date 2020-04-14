#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 12:38:39 2020

@author: mads
"""


import numpy as np


transitions = np.array([[-3], #initial position
                         [5], #first movement
                         [3]]) #second movement


dim = transitions.shape[0]

omega = np.zeros((dim,dim))
xi = np.zeros((dim))

#initial position
omega[0,0] = 1
xi[0] = transitions[0]


for i, trans in enumerate(transitions[1:]):
    omega_i = np.zeros((dim,dim)) #tmp omega matrix
    xi_i = np.zeros((dim)) #tmp xi vector
    
    #fill out the four 
    omega_i[i,i]=1
    omega_i[i+1,i+1]=1
    
    omega_i[i,i+1] = -1
    omega_i[i+1,i] = -1
    
    xi_i[i] = -trans
    xi_i[i+1] = trans

    omega += omega_i
    xi += xi_i
    
mu = np.linalg.inv(omega) @ xi




step={'position':[], 'observations':[]}
steps = []