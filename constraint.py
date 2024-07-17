#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Jul 2024

@author: Noé Fernández González

Copyright 2023 Noé Fernández González <nofergon@alumni.uv.es>
"""

import bpy
import math

if __name__ == "ergoik":
    from importlib import reload
    import particle as prt
    # Needed in version 0.7 to prevent resident modules
    reload(prt)
else:
    from importlib import reload
    import particle as prt
    reload(prt)


class Constraint: 
    """
        Abstract class that defines a constraint
    
    """
    bl_idname = "object.constraint"
    bl_label = "constraint"
    bl_options = {'UNDO'}
    
    def __init__(self):
        """
            Init function
        
        """
        self.particles = []
        self.stiffness = 0.0
        self.k_coef = 0.0
        self.lambda_val = 0.0
        
    def compute_k_coef(self, n):
        """
            Calculates k' in pbd 
        
        """
        # self.k_coef = 1.0 - (1.0 - self.stiffness) ** (1.0 / float(n))
        self.k_coef = 1 / self.stiffness
        self.k_koef = self.k_coef / (n*n)
        #print("Fijamos " + str(n) + " iteraciones   -->  k = " + str(self.stiffness) + "    k' = " + str(self.k_coef) + ".")
        
        
    def proyecta_restriccion(self):
        pass
    

class DistanceConstraint(Constraint):
    """
        Class that extends Constraint to define a distance constraint
    
    """
    bl_idname = "object.distance_constraint"
    bl_label = "distance constraint"
    bl_options = {'UNDO'}
    
    def __init__(self, p1, p2, dist, k):
        """
            Init function
        
        """
        super().__init__()
        self.d = dist
        self.particles.extend([p1, p2])
        self.stiffness = k
        self.k_coef = self.stiffness
        self.lambda_val = 0.0

    def proyecta_restriccion(self):
        """
            Proyects distance constraint in pbd
        
        """
        # get particles involved
        part1 = self.particles[0]
        part2 = self.particles[1]

        # get 1/mass
        w1 = part1.w
        w2 = part2.w

        # check if both inverse masses are different from 0
        if w1 + w2 > 0.0005:
            # get W1 = w1 / (w1 + w2)
            W1 = w1 / (w1 + w2)
            # get W2 = w2 / (w1 + w2)
            W2 = w2 / (w1 + w2)

            # get p1 - p2 and (p1 - p2) / |p1 - p2|
            vd = part1.location - part2.location    
            # get C = |p1 − p2| − d
            C = vd.length - self.d
            
            # get ∆Cp1 = (p1 - p2) / |p1 - p2|
            dC1 = vd.normalized()
            # get ∆Cp2 = (p2 - p1) / |p1 - p2|
            dC2 = -dC1

            # get ∆C = ∆C^2 = ∆Cp1^2 + ∆Cp2^2
            dC = dC1.dot(dC1) + dC2.dot(dC2)

            # Get lambda variations
            # ∆λ1 = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            deltalambda1 = (-C - self.k_coef * self.lambda_val) / (dC * W1 + self.k_coef)
            # ∆λ2 = (−C2 - k' * λ2)/(dC^2 * W2 + k')
            deltalambda2 = (-C - self.k_coef * self.lambda_val) / (dC * W2 + self.k_coef)

            # Get position variations
            # ∆x1 = W1 * dC1 * ∆λ1
            deltap1 = W1 * dC1 * deltalambda1
            # ∆x2 = W2 * dC2 * ∆λ2
            deltap2 = W2 * dC2 * deltalambda2

            # Update lambdas and positions
            if abs(vd.length - self.d) >= 0.0005:
                self.lambda_val += deltalambda1
                self.lambda_val += deltalambda2
                part1.location += deltap1
                part2.location += deltap2
                
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)
            

class TriangleBendingConstraint(Constraint):
    """
        Class that extends Constraint to define a bending constraint
        ref: http://image.diku.dk/kenny/download/kelager.niebe.ea10.pdf
    """
    bl_idname = "object.triangle_bending_constraint"
    bl_label = "triangle bending constraint"
    bl_options = {'UNDO'}
    
    def __init__(self, b0, b1, v, h0, bendk, k):
        """
            Init function
        
        """
        super().__init__()
        self.h0 = h0
        self.particles.extend([b0, b1, v])
        self.bendk = bendk
        self.stiffness = k
        self.k_coef = self.stiffness
        self.lambda_val = 0.0

    def proyecta_restriccion(self):
        """
            Proyects distance constraint in pbd
        
        """
        # get particles involved
        b0 = self.particles[0]
        b1 = self.particles[1]
        v = self.particles[2]

        # get 1/mass
        wb0 = b0.w
        wb1 = b1.w
        wv = v.w
        W = wb0 + wb1 + 2 * wv
        
        # get centroid of the triangle c = (b0 + b1 + v) / 3
        c = (b0.location + b1.location + v.location) / 3
        
        # get h1 = |v - c|, h0 = h0, bendk
        h1 = (v.location - c).length
        h0 = self.h0
        bendk = self.bendk
        
        # check if both inverse masses are different from 0
        if W > 0.0005 and h1 != 0:
            # get C = h1 - (bendk + h0) 
            C = h1 - (bendk + h0)
            
            # get ∆Cb0 = 2wb0 / W * (v - c) * (1 - (bendk + h0) / h1)
            dCb0 = 2 * wb0 / W * (v.location - c) * (1 - (bendk + h0) / h1)
            # get ∆Cb1 = 2wb1 / W * (v - c) * (1 - (bendk + h0) / h1)
            dCb1 = 2 * wb1 / W * (v.location - c) * (1 - (bendk + h0) / h1)
            # get ∆Cv = -4wv / W * (v - c) * (1 - (bendk + h0) / h1)
            dCv = -4 * wv / W * (v.location - c) * (1 - (bendk + h0) / h1)
            
            # # get ∆C = ∆C^2 = ∆Cb0^2 + ∆Cb1^2 + ∆Cv^2
            # dC = dCb0.dot(dCb0) + dCb1.dot(dCb1) + dCv.dot(dCv)

            # # Get lambda variations
            # # ∆λ1 = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            # deltalambdab0 = (-C - self.k_coef * self.lambda_val) / (dC * W1 + self.k_coef)
            # # ∆λ2 = (−C2 - k' * λ2)/(dC^2 * W2 + k')
            # deltalambdab1 = (-C - self.k_coef * self.lambda_val) / (dC * W2 + self.k_coef)
            # # ∆λ3 = (−C3 - k' * λ3)/(dC^2 * W3 + k')
            # deltalambdav = (-C - self.k_coef * self.lambda_val) / (dC * W3 + self.k_coef)

            # # Get position variations
            # # ∆x1 = W1 * dC1 * ∆λ1
            # deltab0 = W1 * dCb0 * deltalambdab0
            # # ∆x2 = W2 * dC2 * ∆λ2
            # deltab1 = W2 * dCb1 * deltalambdab1
            # # ∆x3 = W3 * dC3 * ∆λ3
            # deltav = W3 * dCv * deltalambdav

            # Update lambdas and positions
            # self.lambda_val += deltalambdab0
            # self.lambda_val += deltalambdab1
            # self.lambda_val += deltalambdav
            # b0.location += deltab0
            # b1.location += deltab1
            # v.location += deltav
            b0.location += dCb0
            b1.location += dCb1
            v.location += dCv
                            
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)
        
    def change_bendk(self, bendk):
        self.bendk = bendk
    

class EnvironmentCollisionConstraint(Constraint):
    """
        Class that extends Constraint to define a distance constraint
    
    """
    bl_idname = "object.environmtent_collision_constraint"
    bl_label = "environmtent collision constraint"
    bl_options = {'UNDO'}
    
    def __init__(self, p1, n, dist, k):
        """
            Init function
        
        """
        super().__init__()
        self.d = dist
        self.particles.extend([p1])
        self.stiffness = k
        self.k_coef = self.stiffness
        self.lambda_val = 0.0
        self.n = n

    def proyecta_restriccion(self):
        """
            Proyects distance constraint in pbd
        
        """
        # get particles involved
        part1 = self.particles[0]
        x = part1.location
        
        # get plane normal
        n = self.n
        
        # get 1/mass
        w1 = part1.w

        # check if inverse mass is different from 0
        if w1 > 0.0005:
            # get C = n * x − d
            C = n.dot(x) - self.d
            
            # get ∆Cp1 = n
            dC1 = n

            # get ∆C = ∆C^2 = ∆Cp1^2
            dC = dC1.dot(dC1)

            # Get lambda variation
            # ∆λ1 = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            deltalambda1 = (-C - self.k_coef * self.lambda_val) / (dC * w1 + self.k_coef)
            deltalambda1 *= 0.01
            # Get position variation
            # ∆x1 = W1 * dC1 * ∆λ1
            deltap1 = w1 * dC1 * deltalambda1
     
            if abs(n.dot(x) - self.d) >= 0.0005: 
                # Update lambda and position
                self.lambda_val += deltalambda1 
                part1.location += deltap1
            
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)


