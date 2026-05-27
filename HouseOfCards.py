# -*- coding: utf-8 -*-
# Casa de cartas: N_TRIANGLES /\ + N_TRIANGLES-1 cartas techo

from yade import utils, qt
from yade import *
from math import pi, sin, cos

# =============================================================================
# PARAMETROS
# =============================================================================
N_TRIANGLES = 3

CARD_LENGTH = 63.0e-3        # largo de la carta (eje Z cuando esta parada)
CARD_HEIGHT = 88.0e-3        # lado largo (eje Y parada, eje X tumbada = techo)
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t

DIAMOND_RY = 0.35 * CARD_HEIGHT
DIAMOND_RZ = 0.35 * CARD_LENGTH

# Inclinacion respecto a la vertical: 22.5 deg → triangulos mas agudos que 30 deg.
# Restriccion: TILT <= ~25.7 deg para que las bases no se solapen con x_step=CARD_HEIGHT.
TILT = pi / 8.0              # 22.5 deg

m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)

# =============================================================================
# GRID CENTRADO EN (0,0) — garantiza simetria del rombo
# =============================================================================
N_Y = int(CARD_HEIGHT / (2.0 * SPHERE_RADIUS))   # 11 (impar → centrado en y=0)
N_Z = int(CARD_LENGTH / (2.0 * SPHERE_RADIUS))   # 7  (impar → centrado en z=0)
STEP = 2.0 * SPHERE_RADIUS


def sphere_color(y, z):
	if abs(y) / DIAMOND_RY + abs(z) / DIAMOND_RZ < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	pos_*   : centro de la cara interna en coordenadas mundo.
	angle_z : rotacion alrededor de Z (rad).
	angle_x : rotacion alrededor de X (rad).
	x_local : desplazamiento de las esferas desde la cara interna.
	          Para cartas verticales: +/-SPHERE_RADIUS segun lado.
	          Para la carta techo: +SPHERE_RADIUS (esferas encima de la cara).
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	members = []
	for iy in range(N_Y):
		y = (iy - (N_Y - 1) / 2.0) * STEP
		for iz in range(N_Z):
			z = (iz - (N_Z - 1) / 2.0) * STEP
			wp = rot * Vector3(x_local, y, z) + center
			members.append(sphere(wp, SPHERE_RADIUS,
			                      material=m_card,
			                      color=sphere_color(y, z)))
	O.bodies.appendClumped(members)


# =============================================================================
# GEOMETRIA
# =============================================================================
h_half = (N_Y - 1) * SPHERE_RADIUS          # semialtura del grid = 40 mm

sep_x  = (CARD_HEIGHT / 2.0) * sin(TILT)   # separacion horizontal de cada carta al eje

# Altura del centro de la cara interna para que la esfera inferior toque el suelo:
#   y_world_bottom = y_ctr + r*sin(TILT) - h_half*cos(TILT) = SPHERE_RADIUS
y_ctr = SPHERE_RADIUS * (1.0 - sin(TILT)) + h_half * cos(TILT)

# La separacion entre picos de triangulos vecinos = CARD_HEIGHT (lado largo del techo).
# Esto impone x_step = CARD_HEIGHT.
# Verificacion de no-solapamiento en la base:
#   gap = x_step - 2*(sep_x + r*cos + h_half*sin) - 2*r >= 0
# Con TILT=22.5 deg: gap ≈ 8 mm  ✓
x_step = CARD_HEIGHT

# Altura de la superficie superior de la punta del /\:
#   punta esfera centro: y_ctr + r*sin + h_half*cos = r + 2*h_half*cos
#   punta esfera superficie: 2*r + 2*h_half*cos
y_peak_top = 2.0 * SPHERE_RADIUS + 2.0 * h_half * cos(TILT)

# Para la carta techo:
# Rotacion Rz(pi/2): local(x,y,z) → mundo(-y, x, z)
#   local X → mundo +Y  (espesor vertical ✓)
#   local Y → mundo -X  (lado largo 88mm en eje X ✓)
#   local Z → mundo  Z  (lado corto 63mm en profundidad ✓)
# x_local = +SPHERE_RADIUS: centros a SPHERE_RADIUS por encima de la cara interna.
# La cara interna (abajo) se apoya en la punta del /\.
# pos_y = y_peak_top: la cara inferior de la carta techo descansa en la punta.
# pos_x = xi + x_step/2: la carta cubre exactamente de xi a xi+x_step.

# =============================================================================
# ESCENA
# =============================================================================
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

x_start = -((N_TRIANGLES - 1) / 2.0) * x_step

# Cartas inclinadas: triangulos /\
for i in range(N_TRIANGLES):
	xi = x_start + i * x_step
	add_card(xi - sep_x, y_ctr, 0.0, -TILT, 0.0, -SPHERE_RADIUS)
	add_card(xi + sep_x, y_ctr, 0.0,  TILT, 0.0, +SPHERE_RADIUS)

# Cartas techo: una entre cada par de triangulos vecinos
for i in range(N_TRIANGLES - 1):
	xi    = x_start + i * x_step
	x_mid = xi + x_step / 2.0
	add_card(x_mid, y_peak_top, 0.0, pi / 2.0, 0.0, SPHERE_RADIUS)

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
