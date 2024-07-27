#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Jul 2024

@author: Noé Fernández González

Copyright 2024 Noé Fernández González <nofergon@alumni.uv.es>

"""

import bpy
from mathutils import Vector, Matrix

if __name__ == "xpbd":
    from importlib import reload
    from . import constraint as cns
    from . import particle as prt
    # Needed in version 0.7 to prevent resident modules
    reload(cns)
    reload(prt)
else:
    from importlib import reload
    import constraint as cns
    import particle as prt
    reload(cns)
    reload(prt)

class xpbd_object:
    """
    Class that defines a xpbd object for simulation.
    """
    def __init__(self, obj):
        """
        Initialize xpbd object.

        Args:
            obj (bpy.types.Object): The object to simulate.
        """
        self.obj = obj
        self.particles = []
        self.d_constraints = [] # distance constraints
        self.sh_d_constraints = [] # shear distance constraints
        self.bs_d_constraints = [] # bending spring distance constraints
        self.b_constraints = [] # bending constraints
        self.vb_constraints = [] # vertical bending constraints
        self.hb_constraints = [] # horizontal bending constraints
        self.tb_constraints = [] # triangle bending constraints  
        self.ec_constraints = [] # environmental collisions constraints

    def setup_xpbd(self, context):
        obj = self.obj
        self.d_constraints.clear()
        self.sh_d_constraints.clear()
        self.bs_d_constraints.clear()
        self.b_constraints.clear()
        self.tb_constraints.clear()
        self.ec_constraints.clear() 
        self.particles.clear()

        self.obj.setted_up = True

        if obj.type == 'MESH':
            # Accede a la malla del objeto
            mesh = obj.data

            # Variables para almacenar vértices y aristas
            vertices = []
            edges = []

            # Recorre todos los vértices
            for vertex in mesh.vertices:
                vertices.append(vertex)

            # Recorre todas las aristas
            for edge in mesh.edges:
                edges.append(edge.vertices)
            
            velocity = Vector((0.0, 0.0, 0.0))
    
            for v in vertices:
                p = prt.Particle(v.co, velocity, obj.vdamp)
                self.particles.append(p)

            """ 
                Distance constraints creation
            """
            if obj.distance_constraint:
                # Structural 
                if obj.structural:
                    for e in edges:
                        if mesh.vertices[e[0]] in vertices and mesh.vertices[e[1]] in vertices:
                            i1 = vertices.index(mesh.vertices[e[0]])
                            p1 = self.particles[i1]
                            i2 = vertices.index(mesh.vertices[e[1]])
                            p2 = self.particles[i2]
                            length = (p2.location - p1.location).length
                            c = cns.DistanceConstraint(p1, p2, length, obj.dstiff)
                            c.compute_k_coef(context.scene.niters)
                            self.d_constraints.append(c)
                # Shear
                if obj.shear:
                    for face in mesh.polygons:
                        if len(face.vertices) == 4:
                            v1 = mesh.vertices[face.vertices[0]]
                            v2 = mesh.vertices[face.vertices[1]]
                            v3 = mesh.vertices[face.vertices[2]]
                            v4 = mesh.vertices[face.vertices[3]]

                            i1 = vertices.index(v1)
                            p1 = self.particles[i1]
                            i2 = vertices.index(v2)
                            p2 = self.particles[i2]
                            i3 = vertices.index(v3)
                            p3 = self.particles[i3]
                            i4 = vertices.index(v4)
                            p4 = self.particles[i4]

                            c1 = cns.DistanceConstraint(p1, p3, (p3.location - p1.location).length, obj.shdstiff)
                            c1.compute_k_coef(context.scene.niters)
                            self.sh_d_constraints.append(c1)
                            c2 = cns.DistanceConstraint(p2, p4, (p4.location - p2.location).length, obj.shdstiff)
                            c2.compute_k_coef(context.scene.niters)
                            self.sh_d_constraints.append(c2)

                # Bending spring (each vertex is connected to the next next one)
                if obj.bending_spring:
                    for i in range(len(vertices)):
                        p1 = self.particles[i]
                        p2 = self.particles[(i+1) % len(vertices)]
                        c = cns.DistanceConstraint(p1, p2, (p2.location - p1.location).length, obj.bsdstiff)
                        c.compute_k_coef(context.scene.niters)
                        self.bs_d_constraints.append(c)

                

            """
                Bending constraints creation
            """
            if obj.bending:
                for face in mesh.polygons:
                    if len(face.vertices) == 4:
                        v1 = mesh.vertices[face.vertices[0]]
                        v2 = mesh.vertices[face.vertices[1]]
                        v3 = mesh.vertices[face.vertices[2]]
                        v4 = mesh.vertices[face.vertices[3]]
                        
                        i1 = vertices.index(v1)
                        p1 = self.particles[i1]
                        i2 = vertices.index(v2)
                        p2 = self.particles[i2]
                        i3 = vertices.index(v3)
                        p3 = self.particles[i3]
                        i4 = vertices.index(v4)
                        p4 = self.particles[i4]
                        
                        # Create constraints for each face 
                        c1 = cns.BendingConstraint(p1, p2, p3, p4, obj.bstiff, obj.bend_phi)
                        c1.compute_k_coef(context.scene.niters)
                        self.b_constraints.append(c1)

                        
            """
                Triangle bending constraints creation
            """
            if obj.triangle_bending:
                for face in mesh.polygons:
                    if len(face.vertices) == 4:
                        v1 = mesh.vertices[face.vertices[0]]
                        v2 = mesh.vertices[face.vertices[1]]
                        v3 = mesh.vertices[face.vertices[2]]
                        v4 = mesh.vertices[face.vertices[3]]
                        
                        i1 = vertices.index(v1)
                        p1 = self.particles[i1]
                        i2 = vertices.index(v2)
                        p2 = self.particles[i2]
                        i3 = vertices.index(v3)
                        p3 = self.particles[i3]
                        i4 = vertices.index(v4)
                        p4 = self.particles[i4]
                        
                        # Create constraints for each triangle
                        centroid = (p1.location + p2.location + p3.location) / 3
                        h0 = (p1.location - centroid).length
                        c1 = cns.TriangleBendingConstraint(p2, p3, p1, h0, obj.kbend, obj.tbstiff)
                        c1.compute_k_coef(context.scene.niters)
                        self.tb_constraints.append(c1)
                        
                        centroid = (p2.location + p3.location + p4.location) / 3
                        h0 = (p4.location - centroid).length
                        c2 = cns.TriangleBendingConstraint(p2, p3, p4, h0, obj.kbend, obj.tbstiff)
                        c2.compute_k_coef(context.scene.niters)
                        self.tb_constraints.append(c2)
                        
                        # centroid = (p1.location + p3.location + p4.location) / 3
                        # h0 = (p3.location - centroid).length
                        # c3 = cns.TriangleBendingConstraint(p4, p1, p3, h0, obj.kbend, obj.tbstiff)
                        # c3.compute_k_coef(context.scene.niters)
                        # tb_constraints.append(c3)
                        
                        # centroid = (p1.location + p2.location + p4.location) / 3
                        # h0 = (p2.location - centroid).length
                        # c4 = cns.TriangleBendingConstraint(p1, p4, p2, h0, obj.kbend, obj.tbstiff)
                        # c4.compute_k_coef(context.scene.niters)
                        # tb_constraints.append(c4)

            bpy.ops.object.mode_set(mode = "EDIT")
                
        else:
            print("El objeto no es de tipo MESH")


    def update_xpbd(self, context : bpy.types.Context):
        scene = context.scene
        obj = context.object
        
        for p in self.particles:
            p.force = Vector((obj.force[0],obj.force[1],obj.force[2]))
                
        for p in self.particles:
            p.update(scene.dt)
                    
    def update_pbd_vel(self, dt):
        for p in self.particles:
            p.update_pbd_vel(dt)

    def apply_positions_to_mesh(self):
        # Recorre los vértices y actualiza sus posiciones
        obj = self.obj
        particles = self.particles
        
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='DESELECT')
        bpy.ops.object.mode_set(mode='OBJECT')
        for i, vertice in enumerate(obj.data.vertices):
            if i < len(particles):
                p = particles[i]
                vertice.co = p.location
                # If the option of breaking the mesh is enabled, check if particle velocity is greater than the threshold
                if obj.breakable:
                    if p.velocity.length > obj.break_threshold:
                        vertice.select = True
            
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.delete(type='VERT')
        bpy.ops.object.mode_set(mode='OBJECT')
            
    