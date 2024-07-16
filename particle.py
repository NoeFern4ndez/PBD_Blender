#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Jul 2024

@author: Noé Fernández González
@author: Ignacio García-Fernández

Copyright 2023 Noé Fernández González <nofergon@alumni.uv.es>
Copyright 2023 Ignacio García Fernández <ignacio.garcia@uv.es>
Copyright 2023 IBV - Instituto de Biomecánica de Valencia

"""

import bpy
import mathutils
import random

class Particle:
    """
    Class that defines a particle for simulation.

    Attributes:
        location (mathutils.Vector): The current location of the particle.
        velocity (mathutils.Vector): The current velocity of the particle.
        damping (float): The damping factor for velocity.
    """
    bl_idname = "object.particle"
    bl_label = "particle"
    bl_options = {'UNDO'}
    
    def __init__(self, location: mathutils.Vector, velocity: mathutils.Vector, damping: float):
        """
        Initialize a particle.

        Args:
            location (mathutils.Vector): The initial location of the particle.
            velocity (mathutils.Vector): The initial velocity of the particle.
            damping (float): The damping factor for velocity.
        """
        self.acceleration = mathutils.Vector((0.0, 0.0, 0.0))
        self.force = mathutils.Vector((0.0, 0.0, 0.0))
        self.velocity = velocity.copy()
        self.location = location.copy()
        self.last_location = location.copy()
        self.last_static_location = location.copy()
        self.mass = 1.0
        self.w = 1.0 / self.mass
        self.bloqueada = False
        self.display_size = 0.1
        self.Vdamp = damping

    def set_bloqueada(self, bl):
        if bl:
            self.bloqueada = True
            self.w = 0
            self.mass = float('inf')
            self.acceleration = mathutils.Vector((0.0, 0.0, 0.0))
            self.force = mathutils.Vector((0.0, 0.0, 0.0))
            self.velocity = mathutils.Vector((0.0, 0.0, 0.0))
        else:
            self.bloqueada = False
            self.mass = 1.0
            self.w = 1.0 / self.mass

    def update_pbd_vel(self, dt):
        self.velocity = (self.location.copy() - self.last_location.copy()) / dt
        self.velocity *= self.Vdamp

    def update(self, dt):
        # Actualizar la aceleración de la partícula con la fuerza actual    
        self.acceleration += self.force.copy() * self.w

        # Guardar la posición anterior para PBD
        self.last_location = self.location.copy()

        # Predicción de PBD
        # Utilizar Euler semiimplícito para calcular velocidad y posición
        self.velocity += self.acceleration.copy() * dt
            
        if self.velocity.length < 1e-8:
            self.velocity = mathutils.Vector((0, 0, 0))

        self.location += self.velocity.copy() * dt

        # Limpieza de fuerzas y aceleraciones
        self.acceleration = mathutils.Vector((0.0, 0.0, 0.0))
        self.force = mathutils.Vector((0.0, 0.0, 0.0))

    def set_static_location(self,location):
        self.last_static_location = location
        
