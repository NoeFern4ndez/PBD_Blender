#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Jul 2024

@author: Noé Fernández González

Copyright 2023 Noé Fernández González <nofergon@alumni.uv.es>
"""

import bpy
from mathutils import Vector, Quaternion
import math
import numpy as np
import random

    
bl_info = {
    "name": "PBD",
    "author": "Noé Fernández González",
    "version": (0, 0, 0),
    "blender": (3, 6, 0),
    "location": "3D View > Panel > xpbd",
    "description": "XPBD implementation",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "xpbd"}

if __name__ == "ergoik":
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
    
""" CONSTRAINT ARRAY"""  
d_constraints = [] # distance constraints
sh_d_constraints = [] # shear distance constraints
bs_d_constraints = [] # bending spring distance constraints
b_constraints = [] # bending constraints
vb_constraints = [] # vertical bending constraints
hb_constraints = [] # horizontal bending constraints
tb_constraints = [] # triangle bending constraints  
ec_constraints = [] # environmental collisions constraint
setted_up_objects = []
particles = []

  
""" PANELS """

class xpbdIKMainPanel(bpy.types.Panel):
    """ Implements a Panel to control PBD
    
    Main control panel for xpbd
    """
    bl_label = "xpbd main panel"
    bl_idname = "OBJ_xpbd_main"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "xpbd"

    def draw(self, context : bpy.types.Context) -> None:
        """
        Draw function for the panel

        Parameters
        ----------
        context : bpy.types.Context
            Context in which the panel is created

        Returns
        -------
        None.

        """
        layout = self.layout
        scene = context.scene
        row = layout.row()
        row.prop(scene, "niters")
        row.prop(scene, "dt")
        row = layout.row()
        row.prop(scene, "ik_timer")
        row.prop(context.object, "vdamp")
        
        row = layout.row()
        row.operator("object.setup")
        row = layout.row()
        row.label(text = "Breakable objects")
        row.prop(context.object, "breakable") 
        if context.object.breakable:
            row = layout.row()
            row.prop(context.object, "break_threshold") 
        
        if context.object.setted_up and context.object.mode == "EDIT": 
            row = layout.row()
            row.operator("object.bloq_vertex")  
            row = layout.row()
            row.operator("object.mptv")
            
        row = layout.row() 
        row.label(text = "Constraints to apply")
        row = layout.row() 
        row.prop(context.object, "distance_constraint")
        row.prop(context.object, "bending")
        row = layout.row()
        row.prop(context.object, "triangle_bending")
        row.prop(context.object, "environmental_collisions")
    
        
class xpbdIKConstraintsPanel(bpy.types.Panel):
    """ Implements a Panel to control PBD constraints
    
    Constraints control panel for xpbd
    """
    bl_label = "xpbd constraints panel"
    bl_idname = "OBJ_xpbd_constraints"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "xpbd_constraints"

    def draw(self, context : bpy.types.Context) -> None:
        """
        Draw function for the panel

        Parameters
        ----------
        context : bpy.types.Context
            Context in which the panel is created

        Returns
        -------
        None.

        """
        layout = self.layout
        scene = context.scene
        
        row = layout.row()
        row.prop(context.object, "force") 
        
        if context.object.distance_constraint:
            row = layout.row()
            row.label(text = "Distance constraints")
            row = layout.row()
            row.prop(context.object, "structural")
            if context.object.structural:
                row = layout.row()
                row.prop(context.object, "dstiff", slider = True)  
            row = layout.row()
            row.prop(context.object, "shear")
            if context.object.shear:
                row = layout.row()
                row.prop(context.object, "shdstiff", slider = True)
            row = layout.row()
            row.prop(context.object, "bending_spring")
            if context.object.bending_spring:
                row = layout.row()
                row.prop(context.object, "bsdstiff", slider = True)

        if context.object.bending:
            row = layout.row()
            row.label(text = "Bending constraints")
            row = layout.row()
            row.prop(context.object, "bstiff", slider = True)
            row = layout.row()
            row.prop(context.object, "bend_phi", slider = True)
            
        if context.object.triangle_bending:
            row = layout.row()
            row.label(text = "Triangle bending constraints")
            row = layout.row()
            row.prop(context.object, "tbstiff", slider = True)
            row = layout.row()
            row.prop(context.object, "kbend", slider = True)

        if context.object.environmental_collisions:
            row = layout.row()
            row.label(text = "Environmental collisions constraints")
            row = layout.row()
            row.prop(context.object, "ec_type")
            row = layout.row()
            row.prop(context.object, "ecstiff", slider = True)  
   
        
""" OPERATORS """    
   
class Setup(bpy.types.Operator):
    bl_idname = "object.setup"
    bl_label = "Setup system"

    def execute(self, context : bpy.types.Context):
        obj = context.object
        
        setup_xpbd(context, obj)
        if obj not in setted_up_objects:
            setted_up_objects.append(obj)
        return {'FINISHED'}
    
class Bloq_vertex(bpy.types.Operator):
    bl_idname = "object.bloq_vertex"
    bl_label = "Bloq vertex"

    def execute(self, context : bpy.types.Context):
        bpy.ops.object.mode_set(mode = "OBJECT")
        
        obj = context.object
                
        # Verificar si el objeto es una malla
        if obj and obj.type == 'MESH':
            # Iterar a través de los vértices de la malla
            bpy.ops.object.mode_set(mode = "EDIT")

            i = 0
            for vertex in obj.data.vertices:
                if vertex.select:
                    particles[i].set_bloqueada(not particles[i].bloqueada)
                i += 1
                
        bpy.ops.object.mode_set(mode = "OBJECT")
        bpy.ops.object.mode_set(mode = "EDIT")
        
        return {'FINISHED'}
    
class Move_particle_to_vertex(bpy.types.Operator):
    bl_idname = "object.mptv"
    bl_label = "Move particle to vertex"

    def execute(self, context : bpy.types.Context):
        bpy.ops.object.mode_set(mode = "OBJECT")
        
        obj = context.object
        # Verificar si el objeto es una malla
                 
        if obj and obj.type == 'MESH':
            # Iterar a través de los vértices de la malla
            bpy.ops.object.mode_set(mode = "EDIT")

            i = 0
            for vertex in obj.data.vertices:
                if vertex.select:
                    bloq_state = particles[i].bloqueada
                    particles[i].location = vertex.co
                    particles[i].last_location = vertex.co
                    particles[i].set_bloqueada(True)
                    particles[i].set_bloqueada(bloq_state)
                    
                i += 1
                
        bpy.ops.object.mode_set(mode = "OBJECT")
        bpy.ops.object.mode_set(mode = "EDIT")
        
        return {'FINISHED'}
    
    
""" FUNCTIONS """    
def setup_xpbd(context, obj):
    
    d_constraints.clear()
    sh_d_constraints.clear()
    bs_d_constraints.clear()
    b_constraints.clear()
    tb_constraints.clear()
    ec_constraints.clear() 
    particles.clear()

    obj.setted_up = True
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
            particles.append(p)

        """ 
            Distance constraints creation
        """
        if obj.distance_constraint:
            # Structural 
            if obj.structural:
                for e in edges:
                    if mesh.vertices[e[0]] in vertices and mesh.vertices[e[1]] in vertices:
                        i1 = vertices.index(mesh.vertices[e[0]])
                        p1 = particles[i1]
                        i2 = vertices.index(mesh.vertices[e[1]])
                        p2 = particles[i2]
                        length = (p2.location - p1.location).length
                        c = cns.DistanceConstraint(p1, p2, length, obj.dstiff)
                        c.compute_k_coef(context.scene.niters)
                        d_constraints.append(c)
            # Shear
            if obj.shear:
                for face in mesh.polygons:
                    if len(face.vertices) == 4:
                        v1 = mesh.vertices[face.vertices[0]]
                        v2 = mesh.vertices[face.vertices[1]]
                        v3 = mesh.vertices[face.vertices[2]]
                        v4 = mesh.vertices[face.vertices[3]]

                        i1 = vertices.index(v1)
                        p1 = particles[i1]
                        i2 = vertices.index(v2)
                        p2 = particles[i2]
                        i3 = vertices.index(v3)
                        p3 = particles[i3]
                        i4 = vertices.index(v4)
                        p4 = particles[i4]

                        c1 = cns.DistanceConstraint(p1, p3, (p3.location - p1.location).length, obj.shdstiff)
                        c1.compute_k_coef(context.scene.niters)
                        sh_d_constraints.append(c1)
                        c2 = cns.DistanceConstraint(p2, p4, (p4.location - p2.location).length, obj.shdstiff)
                        c2.compute_k_coef(context.scene.niters)
                        sh_d_constraints.append(c2)

            # Bending spring (each vertex is connected to the next next one)
            if obj.bending_spring:
                for i in range(len(vertices)):
                    p1 = particles[i]
                    p2 = particles[(i+1) % len(vertices)]
                    c = cns.DistanceConstraint(p1, p2, (p2.location - p1.location).length, obj.bsdstiff)
                    c.compute_k_coef(context.scene.niters)
                    bs_d_constraints.append(c)

            

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
                    p1 = particles[i1]
                    i2 = vertices.index(v2)
                    p2 = particles[i2]
                    i3 = vertices.index(v3)
                    p3 = particles[i3]
                    i4 = vertices.index(v4)
                    p4 = particles[i4]
                    
                    # Create constraints for each face 
                    c1 = cns.BendingConstraint(p1, p2, p3, p4, obj.bstiff, obj.bend_phi)
                    c1.compute_k_coef(context.scene.niters)
                    b_constraints.append(c1)

                    
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
                    p1 = particles[i1]
                    i2 = vertices.index(v2)
                    p2 = particles[i2]
                    i3 = vertices.index(v3)
                    p3 = particles[i3]
                    i4 = vertices.index(v4)
                    p4 = particles[i4]
                    
                    # Create constraints for each triangle
                    centroid = (p1.location + p2.location + p3.location) / 3
                    h0 = (p1.location - centroid).length
                    c1 = cns.TriangleBendingConstraint(p2, p3, p1, h0, obj.kbend, obj.tbstiff)
                    c1.compute_k_coef(context.scene.niters)
                    tb_constraints.append(c1)
                    
                    centroid = (p2.location + p3.location + p4.location) / 3
                    h0 = (p4.location - centroid).length
                    c2 = cns.TriangleBendingConstraint(p2, p3, p4, h0, obj.kbend, obj.tbstiff)
                    c2.compute_k_coef(context.scene.niters)
                    tb_constraints.append(c2)
                    
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
               
def update_xpbd(context : bpy.types.Context):
    scene = context.scene
    obj = context.object
    
    for p in particles:
        p.force = Vector((obj.force[0],obj.force[1],obj.force[2]))
            
    for p in particles:
        p.update(scene.dt)

    for c in d_constraints:
        c.lambda_val = 0

    for c in sh_d_constraints:
        c.lambda_val = 0

    for c in bs_d_constraints:
        c.lambda_val = 0

    for c in b_constraints:
        c.lambda_val = 0

    for c in tb_constraints:
        c.lambda_val = 0

    for c in ec_constraints:
        c.lambda_val = 0

    if len(particles) > 0 and obj.environmental_collisions:
        if obj.ec_type == "SPH":
            set_environment_collisions_sphere(context, obj)
        elif obj.ec_type == "BBOX":
            set_environment_collisions_bbox(context, obj)

    for i in range(scene.niters):
        if obj.distance_constraint:
            if obj.structural:
                for c in d_constraints:
                    if c.stiffness != obj.dstiff:
                        c.change_stiff(obj.dstiff, scene.niters)
                    c.proyecta_restriccion()

            if obj.shear:
                for c in sh_d_constraints:
                    if c.stiffness != obj.shdstiff:
                        c.change_stiff(obj.shdstiff, scene.niters)
                    c.proyecta_restriccion()

            if obj.bending_spring:
                for c in bs_d_constraints:
                    if c.stiffness != obj.bsdstiff:
                        c.change_stiff(obj.bsdstiff, scene.niters)
                    c.proyecta_restriccion()

        if obj.bending:
            for c in b_constraints:
                if c.stiffness != obj.bstiff:
                    c.change_stiff(obj.bstiff, scene.niters)
                if c.phi != obj.bend_phi:
                    c.change_phi(obj.bend_phi)
                c.proyecta_restriccion()

        if obj.triangle_bending:
            for c in tb_constraints:
                if c.stiffness != obj.tbstiff:
                    c.change_stiff(obj.tbstiff, scene.niters)
                if c.bendk != obj.kbend:
                    c.change_bendk(obj.kbend)
                c.proyecta_restriccion()

        if obj.environmental_collisions:     
            for c in ec_constraints:
                c.proyecta_restriccion()
        
    for p in particles:
        p.update_pbd_vel(scene.dt)

    # Aplicar las posiciones a la mesh 
    apply_positions_to_mesh(obj)
   
"""
Apply PBD calculated new positions to mesh vertices 
"""
def apply_positions_to_mesh(obj):
    # Recorre los vértices y actualiza sus posiciones
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


def set_environment_collisions_sphere(context, obj):
    scene = context.scene
    ec_constraints.clear()

    # Check every object in the scene
    for ob in bpy.data.objects:
        # Check if the object is a mesh and is not the same object as the one being simulated
        if obj.name != ob.name and ob.type == 'MESH' and "not_collide" not in ob.name:
            # Check if the distance between the object and the particles is less than an arbitrary value (to avoid unnecessary calculations)
            if ((obj.matrix_world @ particles[0].location) - ob.location).length < 20:
                # Calculate the radius of the sphere based on the distance between the origin and a vertex
                radius = (ob.matrix_world @ ob.data.vertices[0].co - ob.location).length
                # Checks if the distance between the particle and the object is less than the radius of the object
                for p in particles:
                    dist = (obj.matrix_world @ p.location - ob.location).length - radius
                    if dist < 0.5:
                        # add a collision constraint where the normal is the vector from the center of the sphere to the particle
                        n = (obj.matrix_world @ particles[0].location - ob.location).normalized()
                        c = cns.EnvironmentCollisionConstraint(p, n, 0, obj.ecstiff)
                        c.compute_k_coef(scene.niters)
                        ec_constraints.append(c)


def set_environment_collisions_bbox(context, obj):
    scene = context.scene
    ec_constraints.clear()
    
    # TODO: change this to only create collisions on desired objects
    for ob in bpy.data.objects:
        if obj.name != ob.name and ob.type == 'MESH'and "not_collide" not in ob.name:
            if ((obj.matrix_world @ particles[0].location) - ob.location).length < 40:
                for p in particles:
                    if point_inside_bbox(ob, obj.matrix_world @ p.location):
                            for face in ob.data.polygons:
                                if ((obj.matrix_world @ p.location) - (ob.matrix_world @ ob.data.vertices[face.vertices[0]].co)).dot(face.normal) < 0.5:
                                        c = cns.EnvironmentCollisionConstraint(p, face.normal, 0, obj.ecstiff)
                                        c.compute_k_coef(scene.niters)
                                        ec_constraints.append(c)
                                
# Función para comprobar si un punto está dentro de la bounding box de un objeto
def point_inside_bbox(obj, point):
    # Obtén la matriz de transformación del objeto como matriz 4x4
    matrix = obj.matrix_world.to_4x4()
    
    # Obtén las coordenadas de la bounding box del objeto en coordenadas locales
    bbox = [Vector(corner) for corner in obj.bound_box]
    
    # Transforma las coordenadas de la bounding box al espacio del mundo
    bbox = [matrix @ corner for corner in bbox]
    
    # Calcula los valores mínimos y máximos en cada eje
    min_x = min(bbox, key=lambda p: p.x).x
    max_x = max(bbox, key=lambda p: p.x).x
    min_y = min(bbox, key=lambda p: p.y).y
    max_y = max(bbox, key=lambda p: p.y).y
    min_z = min(bbox, key=lambda p: p.z).z
    max_z = max(bbox, key=lambda p: p.z).z
    
    # Comprueba si el punto está dentro de la bounding box
    return (min_x <= point.x <= max_x) and (min_y <= point.y <= max_y) and (min_z <= point.z <= max_z)         
        
def move_target(a,b):
    """
        Callback function called each time an object is moved.
        Checks if target object exists, if true, updates its last position property
    
    """
    context = bpy.context
    obj = context.object
            
def target_moved():
    """
        Callback function called each time every time interval.
        Checks if target object exists, if true, executes the simulation loop
    
    """
    context = bpy.context
    obj = context.object
    if context.mode == 'OBJECT':
        if obj.setted_up:
            update_xpbd(context) 

    return bpy.context.scene.ik_timer
        
# List of classes to register. Order is relevant.
# Classes that are used by other classes (e.g. operators
# used by panels) have to be defined first.
classes = [
        xpbdIKMainPanel,
        xpbdIKConstraintsPanel,
        Bloq_vertex,
        Move_particle_to_vertex,
        Setup
    ]

def register():
    """
    Register function for Blender Add-on

    Returns
    -------
    None.

    """

    
    bpy.types.Scene.niters = bpy.props.IntProperty(name = "Constraint proyections Iterations", 
                                                            description = "Number of iterations to solve xpbd", 
                                                            min = 1, 
                                                            default = 120)

    bpy.types.Scene.ik_timer = bpy.props.FloatProperty(name = "IK timer", 
                                                            description = "Timer between ik updates", 
                                                            min = 0.0001, 
                                                            default = 0.01)
    
    bpy.types.Scene.dt = bpy.props.FloatProperty(name = "Time step",
                                                            description = "Time step for the simulation", 
                                                            min = 0.0001, 
                                                            default = 0.02)
    
    bpy.types.Object.vdamp = bpy.props.FloatProperty(name = "Velocity damping",
                                                            description = "Velocity damping factor", 
                                                            min = 0.0001,
                                                            max = 1, 
                                                            default = 0.99)
    
    """ 
        Distance constraint properties
    """
    
    bpy.types.Object.distance_constraint = bpy.props.BoolProperty(name = "Distance constraint",
                                                            description = "Determines if the distance constraints are applied", 
                                                            default = True)
    
    bpy.types.Object.dstiff = bpy.props.FloatProperty(name = "Distance constraint stiff", 
                                                            description = "Stiffness between 0 and 1 of distance constraint", 
                                                            min = 0.0001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.shdstiff = bpy.props.FloatProperty(name = "Shear distance constraint stiff",
                                                            description = "Stiffness between 0 and 1 of shear distance constraint", 
                                                            min = 0.0001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.bsdstiff = bpy.props.FloatProperty(name = "Bending spring distance constraint stiff",
                                                            description = "Stiffness between 0 and 1 of bending spring distance constraint",
                                                            min = 0.0001,
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.structural = bpy.props.BoolProperty(name = "Structural distance constraint",
                                                            description = "Determines if the structural distance constraints are applied", 
                                                            default = True)
    
    bpy.types.Object.shear = bpy.props.BoolProperty(name = "Shear distance constraint",
                                                            description = "Determines if the shear distance constraints are applied", 
                                                            default = True)
    
    bpy.types.Object.bending_spring = bpy.props.BoolProperty(name = "Bending spring distance constraint",
                                                            description = "Determines if the bending spring distance constraints are applied", 
                                                            default = True)
    """
        Bending constraint properties
    """
    
    bpy.types.Object.bending = bpy.props.BoolProperty(name = "Bending constraint",
                                                            description = "Determines if the bending constraints are applied", 
                                                            default = True)
    
    bpy.types.Object.bstiff = bpy.props.FloatProperty(name = "Diagonal Bending constraint stiff",
                                                            description = "Stiffness between 0 and 1 of bending constraint", 
                                                            min = 0.0001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.bend_phi = bpy.props.FloatProperty(name = "Bending angle phi",
                                                        description = "Angle phi for bending constraint",
                                                        min = 0.0,
                                                        max = 180.0,
                                                        default = 0.0)
                                                       
    """
        Triangle bending constraint properties
    """
    
    bpy.types.Object.triangle_bending = bpy.props.BoolProperty(name = "Triangle bending constraint",
                                                            description = "Determines if the triangle bending constraints are applied", 
                                                            default = True)
    
    bpy.types.Object.tbstiff = bpy.props.FloatProperty(name = "Triangle bending constraint stiff",
                                                            description = "Stiffness between 0 and 1 of triangle bending constraint", 
                                                            min = 0.0001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.kbend = bpy.props.FloatProperty(name = "Bending parameter",
                                                        description = "Bending parameter for triangle bending constraint",
                                                        min = 0.0,
                                                        max = 1.0,
                                                        default = 0.0)

    """
        Environment collisions constraint properties
    """
    bpy.types.Object.environmental_collisions = bpy.props.BoolProperty(name = "Environmental collisions constraint",
                                                            description = "Determines if the environmental collisions constraints are applied", 
                                                            default = True)

    bpy.types.Object.ecstiff = bpy.props.FloatProperty(name = "Environment collisions constraint stiff",
                                                            description = "Stiffness between 0 and 1 of environment collisions constraint", 
                                                            min = 0.0001, 
                                                            max = 1,
                                                            default = 0.5)
    
    env_coll_type = [
                ("BBOX","bounding_box","Bounding box based collision",1),
                ("SPH","sphere","Sphere based collision",2),
                ("NONE","none","No rotation constraints",4)
                ]
    bpy.types.Object.ec_type = bpy.props.EnumProperty(
        items = env_coll_type,
        name = "Environmental Collisions Type",
        description = "Type of collision to apply",
        default = "SPH",
        )
    

    """
    """
    
    bpy.types.Object.force = bpy.props.FloatVectorProperty(name = "External forces",
                                                            description = "Force to apply in X, Y and Z axis", 
                                                            size = 3,
                                                            default = (0.0, 0.0, -9.81))
    
    bpy.types.Object.setted_up = bpy.props.BoolProperty(name = "Object setted up",
                                                            description = "Checks if an object has been setted up", 
                                                            default = False)
    
    bpy.types.Object.breakable = bpy.props.BoolProperty(name = "Breakable",
                                                            description = "Determines if the meshes are breakable or not", 
                                                            default = False)
    
    bpy.types.Object.break_threshold = bpy.props.FloatProperty(name = "Break threshold",
                                                            description = "Determines the threshold to break a mesh", 
                                                            min = 0.00000001,
                                                            default = 200)
    
    # CLASSES
    for cl in classes:
        bpy.utils.register_class(cl)
        
    # Callback functions
    bpy.app.handlers.depsgraph_update_post.append(move_target)

    bpy.app.timers.register(target_moved)

        
        
def unregister():
    """
    Unregister function for Blender Add-on

    Returns
    -------
    None.

    """
    # CLASSES
    for cl in classes:
        bpy.utils.unregister_class(cl)

    del bpy.types.Scene.niters
    del bpy.types.Object.last_pos
    del bpy.types.Object.xforce
    del bpy.types.Object.yforce
    del bpy.types.Object.zforce
    del bpy.types.Scene.ik_timer
    del bpy.types.Object.stiff
    del bpy.app.handlers.depsgraph_update_post
    bpy.app.timers.unregister(target_moved)
    
if __name__ == "__main__":
    register()