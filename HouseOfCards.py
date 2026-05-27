# -*- coding: utf-8 -*-
# Casa de cartas: N_TRIANGLES pares /\ en fila horizontal (eje X)

from yade import utils, qt
from yade import *
from math import pi, sin, cos

# =============================================================================
# PARAMETROS
# =============================================================================
N_TRIANGLES = 3

CARD_LENGTH = 63.0e-3        # eje Z (largo, profundidad de la carta)
CARD_HEIGHT = 88.0e-3        # eje Y (alto, contra la gravedad)
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t

DIAMOND_RY = 0.35 * CARD_HEIGHT   # semieje vertical del rombo
DIAMOND_RZ = 0.35 * CARD_LENGTH   # semieje horizontal del rombo

TILT = pi / 6.0              # inclinacion respecto a la vertical (30 deg)

m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)

# =============================================================================
# GRID DE ESFERAS CENTRADO EN (0,0) — garantiza simetria del rombo
# =============================================================================
N_Y = int(CARD_HEIGHT / (2.0 * SPHERE_RADIUS))   # = 11  (impar → centrado en y=0)
N_Z = int(CARD_LENGTH / (2.0 * SPHERE_RADIUS))   # = 7   (impar → centrado en z=0)
STEP = 2.0 * SPHERE_RADIUS


def sphere_color(y, z):
	# rombo L1 con semiejes RY, RZ, centrado en (0,0)
	if abs(y) / DIAMOND_RY + abs(z) / DIAMOND_RZ < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	pos_*   : centro de la cara interna en coordenadas mundo.
	angle_z : inclinacion alrededor de Z (rad).
	angle_x : rotacion alrededor de X (rad).
	x_local : +-SPHERE_RADIUS segun lado del triangulo.
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	members = []
	for iy in range(N_Y):
		y = (iy - (N_Y - 1) / 2.0) * STEP   # centrado en y=0
		for iz in range(N_Z):
			z = (iz - (N_Z - 1) / 2.0) * STEP  # centrado en z=0
			wp = rot * Vector3(x_local, y, z) + center
			members.append(sphere(wp, SPHERE_RADIUS,
			                      material=m_card,
			                      color=sphere_color(y, z)))
	O.bodies.appendClumped(members)


# =============================================================================
# GEOMETRIA: /\
# =============================================================================
# sep_x: distancia en X del centro de cada carta a x=0
sep_x = (CARD_HEIGHT / 2.0) * sin(TILT)

# h_half: semialtura del grid (extremo inferior de las esferas en local)
h_half = (N_Y - 1) * SPHERE_RADIUS   # = 40 mm

# y_ctr: altura del centro de la cara interna para que el borde inferior
# de las esferas quede en y = SPHERE_RADIUS (rozando el suelo)
#   y_world_bottom = y_ctr + r*sin(TILT) - h_half*cos(TILT) = r
y_ctr = SPHERE_RADIUS * (1.0 - sin(TILT)) + h_half * cos(TILT)

# x_step: separacion entre centros de triangulos para evitar solapamiento.
# La base de la carta inclinada llega hasta:
#   x_max = sep_x + r*cos(TILT) + h_half*sin(TILT) + r   (superficie exterior)
# Dos cartas adyacentes no se solapan si x_step >= 2 * x_max.
# Agregamos 4*r de holgura.
x_max_card = sep_x + SPHERE_RADIUS * (1.0 + cos(TILT)) + h_half * sin(TILT)
x_step = 2.0 * x_max_card + 4.0 * SPHERE_RADIUS

# =============================================================================
# ESCENA
# =============================================================================
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

x_start = -((N_TRIANGLES - 1) / 2.0) * x_step

for i in range(N_TRIANGLES):
	xi = x_start + i * x_step
	add_card(xi - sep_x, y_ctr, 0.0, -TILT, 0.0, -SPHERE_RADIUS)
	add_card(xi + sep_x, y_ctr, 0.0,  TILT, 0.0, +SPHERE_RADIUS)

# =============================================================================
# MOTORES
# =============================================================================
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
