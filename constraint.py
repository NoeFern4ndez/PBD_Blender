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

        self.k_coef = 1.0 - (1.0 - self.stiffness) ** (1.0 / float(n))
        # self.k_coef = 1 / self.stiffness
        # self.k_koef = self.k_coef / (n*n)
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

        # get p1 - p2 
        v = part1.location - part2.location    
        # get C = |p1 − p2| − d
        C = v.length - self.d
        
        # check if constraint is satisfied
        if C > 0.0001 or C < -0.0001:
            # get ∆Cp1 = (p1 - p2) / |p1 - p2|
            dC1 = v.normalized()
            # get ∆Cp2 = (p2 - p1) / |p1 - p2|
            dC2 = -dC1

            # Get lambda variation
            # ∆λ = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            lambdaDenom = dC1.dot(dC1) * w1 + dC2.dot(dC2) * w2 + self.k_coef
            if lambdaDenom != 0:
                deltalambda = (-C - self.k_coef * self.lambda_val) / lambdaDenom

                # Get position variations
                # ∆x1 = W1 * dC1 * ∆λ1
                deltap1 = w1 * dC1 * deltalambda
                # ∆x2 = W2 * dC2 * ∆λ2
                deltap2 = w2 * dC2 * deltalambda

                # Update lambda and positions
                self.lambda_val += deltalambda
                part1.location += deltap1
                part2.location += deltap2

                
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)
            
class BendingConstraint(Constraint):
    """
        Class that extends Constraint to define a bending constraint
        ref: https://matthias-research.github.io/pages/publications/posBasedDyn.pdf
    """
    bl_idname = "object.bending_constraint"
    bl_label = "bending constraint"
    bl_options = {'UNDO'}
    
    def __init__(self, p1, p2, p3, p4, phi, k):
        """
            Init function
        
        """
        super().__init__()
        self.particles.extend([p1, p2, p3, p4])
        self.phi = math.radians(phi)
        self.stiffness = k
        self.k_coef = self.stiffness
        self.lambda_val = 0.0

    def proyecta_restriccion(self):
        """
            Proyects bending constraint in pbd
        
        """
        # get particles involved
        p1 = self.particles[0]
        p2 = self.particles[1]
        p3 = self.particles[2]
        p4 = self.particles[3]

        l1 = p1.location
        l2 = p2.location
        l3 = p3.location
        l4 = p4.location

        # get 1/mass
        w1 = p1.w
        w2 = p2.w
        w3 = p3.w
        w4 = p4.w

        # Get edge vectors em = l2 - l1, el = l3 - l1, er = l4 - l1
        em = l2 - l1
        el = l3 - l1
        er = l4 - l1

        # Get normals of the triangles n1 = p2 x p3 / |p2 x p3| and n2 = p2 x p4 / |p2 x p4|
        n1 = em.cross(el)
        n2 = em.cross(er)
        nl1 = n1.length
        nl2 = n2.length

        if nl1 != 0 and nl2 != 0:
            # Normalize normals
            n1 = n1 / nl1
            n2 = n2 / nl2

            # Get phi = arccos(n1 . n2)
            cosphi = n1.dot(n2)
            # force cosphi to be in the range of [-1, 1]
            cosphi = min(1, max(-1, cosphi))
            phi = math.acos(cosphi)

            # get C = phi - phi0
            C = phi - self.phi

            # check if constraint is satisfied
            if C > 0.0001 or C < -0.0001:
                # get denominator for the derivative: -sqrt(1 - cosphi^2)
                denom = -math.sqrt(1 - cosphi * cosphi)
                if denom != 0:
                    # ∆Cp2 = ((er x n1 + cosphi * n2 x er) / nl2 + (el x n2 + cosphi * n1 x el) / nl1) / denom
                    dC2 = ((er.cross(n1) + cosphi * n2.cross(er)) / nl2 + (el.cross(n2) 
                    + cosphi * n1.cross(el)) / nl1) / denom
                    # ∆Cp3 = (n2 x em - cosphi * n1 x em) / nl1 / denom
                    dC3 = (n2.cross(em) - cosphi * n1.cross(em)) / nl1 / denom
                    # ∆Cp4 = (n1 x em - cosphi * n2 x em) / nl2 / denom
                    dC4 = (n1.cross(em) - cosphi * n2.cross(em)) / nl2 / denom
                    # ∆Cp1 = - ∆Cp2 - ∆Cp3 - ∆Cp4
                    dC1 = -dC2 - dC3 - dC4

                    # Get lambda variation
                    # ∆λ = (−C - k' * λ)/(dC^2 * W + k')
                    lambdaDenom = w1 * dC1.dot(dC1) + w2 * dC2.dot(dC2) + w3 * dC3.dot(dC3) + w4 * dC4.dot(dC4) + self.k_coef
                    if lambdaDenom != 0:
                        deltalambda = (-C - self.k_coef * self.lambda_val)  / lambdaDenom
        
                        # Get position variations
                        # ∆x1 = w1 * dC1 * ∆λ
                        deltap1 = w1 * dC1 * deltalambda
                        # ∆x2 = w2 * dC2 * ∆λ
                        deltap2 = w2 * dC2 * deltalambda
                        # ∆x3 = w3 * dC3 * ∆λ
                        deltap3 = w3 * dC3 * deltalambda
                        # ∆x4 = w4 * dC4 * ∆λ
                        deltap4 = w4 * dC4 * deltalambda

                        # Update lambda and positions
                        self.lambda_val += deltalambda
                        p1.location += deltap1
                        p2.location += deltap2
                        p3.location += deltap3
                        p4.location += deltap4
                            
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)
        
    def change_phi(self, phi):
        self.phi = math.radians(phi)

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
            W1 = wb0 / W
            W2 = wb1 / W
            W3 = wv / W

            # get C = h1 - (bendk + h0) 
            C = h1 - (bendk + h0)
            
            # get ∆Cb0 = 2wb0 / W * (v - c) * (1 - (bendk + h0) / h1)
            dCb0 = 2 * wb0 / W * (v.location - c) * (1 - (bendk + h0) / h1)
            # get ∆Cb1 = 2wb1 / W * (v - c) * (1 - (bendk + h0) / h1)
            dCb1 = 2 * wb1 / W * (v.location - c) * (1 - (bendk + h0) / h1)
            # get ∆Cv = -4wv / W * (v - c) * (1 - (bendk + h0) / h1)
            dCv = -4 * wv / W * (v.location - c) * (1 - (bendk + h0) / h1)
            
            # get ∆C = ∆C^2 = ∆Cb0^2 + ∆Cb1^2 + ∆Cv^2
            dC = dCb0.dot(dCb0) + dCb1.dot(dCb1) + dCv.dot(dCv)

            # Get lambda variations: eq 17: ∆λ = (−C - k' * λ)/(dC^2 * W + k')
            # ∆λ1 = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            deltalambdab0 = (-C - self.k_coef * self.lambda_val) / (dC * W1 + self.k_coef)
            # ∆λ2 = (−C2 - k' * λ2)/(dC^2 * W2 + k')
            deltalambdab1 = (-C - self.k_coef * self.lambda_val) / (dC * W2 + self.k_coef)
            # ∆λ3 = (−C3 - k' * λ3)/(dC^2 * W3 + k')
            deltalambdav = (-C - self.k_coef * self.lambda_val) / (dC * W3 + self.k_coef)

            # Get position variations: eq 18: ∆x = W * dC * ∆λ
            # ∆x1 = W1 * dC1 * ∆λ1
            deltab0 = W1 * dCb0 * deltalambdab0
            # ∆x2 = W2 * dC2 * ∆λ2
            deltab1 = W2 * dCb1 * deltalambdab1
            # ∆x3 = W3 * dC3 * ∆λ3
            deltav = W3 * dCv * deltalambdav

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
            Proyects environmental constraint in pbd
        
        """
        # get particles involved
        part1 = self.particles[0]
        x = part1.location
        
        # get plane normal
        n = self.n
        
        # get 1/mass
        w1 = part1.w

        # get C = n * x − d
        C = n.dot(x) - self.d
            
        if C > 0.0001 or C < -0.0001:
            # get ∆Cp1 = n
            dC1 = n

            # Get lambda variation
            # ∆λ1 = (−C1 - k' * λ1)/(dC^2 * W1 + k')
            lambdaDenom = dC1.dot(dC1) * w1 + self.k_coef + 1000
            if lambdaDenom != 0:
                deltalambda = (-C - self.k_coef * self.lambda_val) / lambdaDenom

                # Get position variation
                # ∆x1 = W1 * dC1 * ∆λ1
                deltap1 = w1 * dC1 * deltalambda
            
                # Update lambda and position
                self.lambda_val += deltalambda
                part1.location += deltap1
            
    def change_stiff(self, stiff, niters):
        self.stiffness = stiff
        self.compute_k_coef(niters)


