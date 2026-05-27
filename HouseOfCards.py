# -*- coding: utf-8 -*-
# Dos cartas rigidas (clump) - /\ con espesor en X, gravedad en -Y

from yade import utils, qt
from yade import *
from math import pi, sin, cos, sqrt

# --- geometria (m) ---
CARD_LENGTH = 63.0e-3      # eje Z (largo, profundidad)
CARD_HEIGHT = 88.0e-3      # eje Y (alto, con gravedad en -Y)
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t: diametro = espesor simulado
DIAMOND_RZ = 0.10 * CARD_LENGTH
DIAMOND_RY = 0.10 * CARD_HEIGHT

# --- fisica ---
m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)


def sphere_color(y_face, z_face):
	cy = y_face / DIAMOND_RY
	cz = z_face / DIAMOND_RZ
	if abs(cy + cz) + abs(cy - cz) < 2.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	Genera una carta como clump de esferas.

	pos_*   : centro geometrico de la cara interna (x=0 local) en el mundo.
	angle_z : inclinacion alrededor de Z (rad). >0 inclina la carta hacia -X.
	angle_x : rotacion alrededor de X (rad). 0 = carta paralela al plano XY global.
	x_local : offset del centro de esferas respecto a la cara interna.
	          -SPHERE_RADIUS = cuerpo a la izquierda (carta izquierda del /\ )
	          +SPHERE_RADIUS = cuerpo a la derecha  (carta derecha  del /\ )

	Marco local: cara interna en x=0, eje Y = alto, eje Z = largo.
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	step = 2.0 * SPHERE_RADIUS
	members = []
	y = -CARD_HEIGHT / 2.0 + SPHERE_RADIUS
	while y <= CARD_HEIGHT / 2.0 - SPHERE_RADIUS + 1e-9:
		z = -CARD_LENGTH / 2.0 + SPHERE_RADIUS
		while z <= CARD_LENGTH / 2.0 - SPHERE_RADIUS + 1e-9:
			local = Vector3(x_local, y, z)
			wp = rot * local + center
			members.append(
				sphere(wp, SPHERE_RADIUS, material=m_card,
				       color=sphere_color(y, z))
			)
			z += step
		y += step
	O.bodies.appendClumped(members)


O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

# /\ : cara interna de cada carta en x_world=0 al techo
# El cuerpo de la carta izq. queda a la izquierda  (x_local = -r)
# El cuerpo de la carta der. queda a la derecha (x_local = +r)
tilt  = pi / 6.0
sep_x = (CARD_HEIGHT / 2.0) * sin(tilt)

# y_ctr: el centro geometrico de la cara interna tal que la
# esfera inferior toque el suelo (world y = SPHERE_RADIUS)
# Deducido de la cinematica de la rotacion:
y_ctr = (SPHERE_RADIUS * (1.0 - sin(tilt) - cos(tilt))
         + (CARD_HEIGHT / 2.0) * cos(tilt))

add_card(-sep_x, y_ctr, 0.0, -tilt, 0.0, -SPHERE_RADIUS)
add_card( sep_x, y_ctr, 0.0,  tilt, 0.0, +SPHERE_RADIUS)

O.engines = [
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Wall_Aabb()]),
	InteractionLoop(
		[Ig2_Sphere_Sphere_ScGeom(), Ig2_Wall_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_FrictPhys()],
		[Law2_ScGeom_FrictPhys_CundallStrack()],
	),
	NewtonIntegrator(damping=0.35, gravity=(0, -9.81, 0)),
]
O.dt = utils.PWaveTimeStep() * 0.4

V = qt.View()
V.viewDir = (0, 0, -1)
V.upVector = (0, 1, 0)
V.center()
