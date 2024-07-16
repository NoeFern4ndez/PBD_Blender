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
    "name": "ERGO-IK",
    "author": "Noé Fernández González",
    "version": (0, 0, 0),
    "blender": (3, 6, 0),
    "location": "3D View > Panel > xpbdik",
    "description": "Inverse kinematics solver using xpbd",
    "warning": "",
    "wiki_url": "",
    "tracker_url": "",
    "category": "IK"}

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
ec_constraints = [] # environmental collisions constraint
particles = []
initial_particles = []
lengths = []
Vdamp = 0.9999
dt = 0.02
  
""" PANELS """

class xpbdIKMainPanel(bpy.types.Panel):
    """ Implements a Panel to control inverse kinematics
    
    Main control panel for xpbdIK
    """
    bl_label = "xpbdIK control panel"
    bl_idname = "ARMATURE_PT_xpbdIK"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "xpbdIK"

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
        row.prop(scene, "ik_timer")
        """
        row = layout.row()
        row.prop(context.object, "max_angle")
        """
        
        row = layout.row()
        row.prop(context.object, "stiff")
        # row.prop(context.object, "vstiff")
        
        row = layout.row()
        row.operator("object.setup")
        row = layout.row()
        row.label(text = "Breakable objects")
        row.prop(context.object, "breakable")     
        
        if context.object.setted_up and context.object.mode == "EDIT": 
            row = layout.row()
            row.operator("object.bloq_vertex")  
            row = layout.row()
            row.operator("object.mptv")
            
        row = layout.row()
        row.label(text = "Force to apply")
        row.prop(context.object, "xforce")     
        row.prop(context.object, "yforce")     
        row.prop(context.object, "zforce")     
   
        
""" OPERATORS """    
   
class Setup(bpy.types.Operator):
    bl_idname = "object.setup"
    bl_label = "Setup system"

    def execute(self, context : bpy.types.Context):
        obj = context.object
        
        setup_xpbd(context, obj)
        
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
    particles.clear()
    initial_particles.clear()
    lengths.clear()

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
            p = prt.Particle(v.co, velocity, Vdamp)
            """
            pi = prt.Particle(v.co, velocity, Vdamp)
            pi.set_bloqueada(True)
            """
            particles.append(p)
            """
            initial_particles.append(pi)
            c = cns.DistanceConstraint(pi, p, 0, obj.vstiff)
            c.compute_k_coef(context.scene.niters)
            v_constraints.append(c)
            """
            
        for e in edges:
            if mesh.vertices[e[0]] in vertices and mesh.vertices[e[1]] in vertices:
                i1 = vertices.index(mesh.vertices[e[0]])
                p1 = particles[i1]
                # pi1 = initial_particles[i1]
                i2 = vertices.index(mesh.vertices[e[1]])
                p2 = particles[i2]
                # pi2 = initial_particles[i2]
                length = (p2.location - p1.location).length
                c = cns.DistanceConstraint(p1, p2, length, obj.stiff)
                c.compute_k_coef(context.scene.niters)
                d_constraints.append(c)
                """
                c = cns.DistanceConstraint(pi1, pi2, length, obj.stiff)
                c.compute_k_coef(context.scene.niters)
                d_constraints.append(c)
                """
                
        bpy.ops.object.mode_set(mode = "EDIT")
               
    else:
        print("El objeto no es de tipo MESH")
               
def update_xpbd(context : bpy.types.Context):
    scene = context.scene
    obj = context.object
    
    if len(particles) > 0:
        set_environment_collisions(context, obj)
    
    for p in particles:
        p.force = Vector((obj.xforce,obj.yforce,obj.zforce))
            
    for p in particles:
        p.update(dt)

    for c in d_constraints:
        c.lambda_val = 0

    for c in ec_constraints:
        c.lambda_val = 0

    for i in range(scene.niters):
        for c in d_constraints:
            if c.stiffness != obj.stiff:
                c.change_stiff(obj.stiff, scene.niters)
            c.proyecta_restriccion()
                
        for c in ec_constraints:
            c.proyecta_restriccion()
            
    """     
    for i in range(1):    
        for c in v_constraints:
            if c.stiffness != obj.vstiff:
                c.change_stiff(obj.vstiff, scene.niters)
            c.proyecta_restriccion()
    """
        
    for p in particles:
        p.update_pbd_vel(dt)

    # Aplicar las posiciones a la mesh 
    apply_positions_to_mesh(obj)
   
def apply_positions_to_mesh(obj):
    # Recorre los vértices y actualiza sus posiciones
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='DESELECT')
    bpy.ops.object.mode_set(mode='OBJECT')
    for i, vertice in enumerate(obj.data.vertices):
        if i < len(particles):
            p = particles[i]
            vertice.co = p.location
            if obj.breakable:
                if p.velocity.length > 200:
                    vertice.select = True
        
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.delete(type='VERT')
    bpy.ops.object.mode_set(mode='OBJECT')


def set_environment_collisions(context, obj):
    scene = context.scene
    ec_constraints.clear()
    
    # TODO: change this to only create collisions on desired objects
    for ob in bpy.data.objects:
        if obj.name != ob.name and ob.type == 'MESH':
            if ((obj.matrix_world @ particles[0].location) - ob.location).length < 40:
                for p in particles:
                    if point_inside_bbox(ob, obj.matrix_world @ p.location):
                            for face in ob.data.polygons:
                                if ((obj.matrix_world @ p.location) - (ob.matrix_world @ ob.data.vertices[face.vertices[0]].co)).dot(face.normal) < 0.5:
                                            c = cns.EnvironmentCollisionConstraint(p, face.normal, 0, 1)
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


"""
def set_environment_collisions(context, obj):
    scene = context.scene
    ec_constraints.clear()
    
    for p in particles:
        ray_dir = p.velocity.normalized()
            
        origin = p.location
        hit, location, normal, face_index = obj.ray_cast(origin, ray_dir)
        
        if hit: 
            c = cns.EnvironmentCollisionConstraint(p, normal, 0, obj.stiff)
            c.compute_k_coef(scene.niters)
            ec_constraints.append(c)
"""
            
           
        
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
    
    bpy.types.Object.max_angle = bpy.props.IntProperty(name = "Angle constraint º", 
                                                            description = "Angle set as maximum in angular constraint", 
                                                            min = 1, 
                                                            max = 180,
                                                            default = 120)
    
    bpy.types.Scene.ik_timer = bpy.props.FloatProperty(name = "IK timer", 
                                                            description = "Timer between ik updates", 
                                                            min = 0.0001, 
                                                            default = 0.01)
    
    bpy.types.Object.stiff = bpy.props.FloatProperty(name = "Distance constraint stiff", 
                                                            description = "Stiffness between 0 and 1 of distance constraint", 
                                                            min = 0.00000001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.xforce = bpy.props.FloatProperty(name = "X Force", 
                                                            description = "Force to apply in X axis", 
                                                            default = 0.0)
    
    bpy.types.Object.yforce = bpy.props.FloatProperty(name = "Y Force", 
                                                            description = "Force to apply in Y axis", 
                                                            default = 0.0)
    
    bpy.types.Object.zforce = bpy.props.FloatProperty(name = "Z Force", 
                                                            description = "Force to apply in Z axis", 
                                                            default = -9.81)
    
    bpy.types.Object.vstiff = bpy.props.FloatProperty(name = "Volume constraint stiff", 
                                                            description = "Stiffness between 0 and 1 of volume constraint", 
                                                            min = 0.00000001, 
                                                            max = 1,
                                                            default = 0.5)
    
    bpy.types.Object.setted_up = bpy.props.BoolProperty(name = "Object setted up",
                                                            description = "Checks if an object has been setted up", 
                                                            default = False)
    
    bpy.types.Object.breakable = bpy.props.BoolProperty(name = "Breakable",
                                                            description = "Determines if the meshes are breakable or not", 
                                                            default = False)
    
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

    """
    del bpy.types.Armature.ergoik_grips_hidden
    del bpy.types.Armature.ergoik_reach_influence
    
    # DRIVERS
    del bpy.app.driver_namespace['get_stiff']
    """
    del bpy.types.Scene.niters
    del bpy.types.Object.max_angle
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