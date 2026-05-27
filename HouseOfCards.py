# -*- coding: utf-8 -*-
# Casa de cartas: piramide completa de N_TRIANGLES niveles

from yade import utils, qt
from yade import *
from math import pi, sin, cos

# =============================================================================
# PARAMETROS
# =============================================================================
N_TRIANGLES = 3              # triangulos en la base; la piramide tendra N niveles

CARD_LENGTH = 63.0e-3        # lado corto (eje Z cuando la carta esta parada)
CARD_HEIGHT = 88.0e-3        # lado largo (eje Y parada, eje X tumbada = techo)
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t

DIAMOND_RY = 0.35 * CARD_HEIGHT
DIAMOND_RZ = 0.35 * CARD_LENGTH

# Inclinacion respecto a la vertical (22.5 deg).
# Restriccion para no solapar bases con x_step = CARD_HEIGHT: TILT <= ~25.7 deg.
TILT = pi / 8.0              # 22.5 deg — triangulos agudos

m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)

# =============================================================================
# GRID DE ESFERAS CENTRADO EN (0,0)
# =============================================================================
N_Y = int(CARD_HEIGHT / (2.0 * SPHERE_RADIUS))   # 11 esferas en alto
N_Z = int(CARD_LENGTH / (2.0 * SPHERE_RADIUS))   # 7  esferas en largo
STEP = 2.0 * SPHERE_RADIUS

# =============================================================================
# GEOMETRIA DERIVADA
# =============================================================================
h_half = (N_Y - 1) * SPHERE_RADIUS          # semialtura del grid (40 mm)
sep_x  = (CARD_HEIGHT / 2.0) * sin(TILT)   # separacion horizontal carta-eje

# Separacion entre centros de triangulos vecinos = lado largo de la carta techo.
# Con TILT=22.5 deg el hueco entre bases adyacentes es ~8 mm (sin solapamiento).
x_step = CARD_HEIGHT

# =============================================================================
# FUNCIONES
# =============================================================================
def sphere_color(y, z):
	if abs(y) / DIAMOND_RY + abs(z) / DIAMOND_RZ < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	Crea una carta rigida (clump de esferas).

	pos_*   : centro de la cara interna en coordenadas mundo.
	angle_z : rotacion alrededor de Z mundo (rad).
	angle_x : rotacion alrededor de X local (rad).
	x_local : +-r para cartas verticales; +r para carta techo.
	          Define de que lado de la cara interna quedan las esferas.

	Carta techo (angle_z=pi/2, x_local=+r):
	  Rz(pi/2): local(x,y,z) -> mundo(-y, x, z)
	    local X -> mundo +Y  (espesor vertical)
	    local Y -> mundo -X  (lado largo 88 mm cubre eje X)
	    local Z -> mundo  Z  (lado corto 63 mm en profundidad)
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
# RECURRENCIA DE ALTURAS POR NIVEL
# =============================================================================
# Nivel k descansa sobre el techo del nivel k-1.
#
# y_ctr_k : altura del centro de la cara interna de las cartas del nivel k,
#            calculada para que la esfera inferior del /\ toque la superficie
#            del techo del nivel k-1 (o el suelo para k=0).
#
# Derivacion:
#   Esfera inferior carta izquierda (local (-r,-h_half,0), rot -TILT):
#     y_mundo = y_ctr + r*sin(TILT) - h_half*cos(TILT)
#   Esta debe ser = y_floor_k + r  (center de esfera sobre el suelo de nivel k)
#   => y_ctr = y_floor_k + r*(1-sin) + h_half*cos
#
#   Punta superior del /\ nivel k (superficie):
#     y_peak_top_k = y_ctr_k + r*(1+sin) + h_half*cos
#
#   El techo del nivel k tiene cara inferior en y_peak_top_k
#   y superficie superior en y_peak_top_k + 2*r.
#   Luego: y_floor_{k+1} = y_peak_top_k + 2*r
#
#   Recurrencia:
#     y_ctr_{k+1} = y_ctr_k + 4*r + 2*h_half*cos(TILT)

y_ctr_0 = SPHERE_RADIUS * (1.0 - sin(TILT)) + h_half * cos(TILT)

level_y_ctr  = []   # altura y_ctr para triangulos de cada nivel
level_y_roof = []   # altura y de la cara interna del techo de cada nivel

y_ctr_k = y_ctr_0
for k in range(N_TRIANGLES):
	level_y_ctr.append(y_ctr_k)
	y_peak_top_k = y_ctr_k + SPHERE_RADIUS * (1.0 + sin(TILT)) + h_half * cos(TILT)
	level_y_roof.append(y_peak_top_k)
	# recurrencia al siguiente nivel
	y_ctr_k = y_ctr_k + 4.0 * SPHERE_RADIUS + 2.0 * h_half * cos(TILT)

# =============================================================================
# ESCENA
# =============================================================================
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

# Origen X del nivel 0 (fila de N_TRIANGLES triangulos centrada en x=0)
x_start = -((N_TRIANGLES - 1) / 2.0) * x_step

# Posicion X de los triangulos en el nivel k:
#   x_start + k*(x_step/2) + i*x_step,  i = 0 .. N_TRIANGLES-k-1
# Cada nivel se desplaza x_step/2 para quedar entre los del nivel anterior.
# Posicion X de los techos del nivel k:
#   x_start + k*(x_step/2) + i*x_step + x_step/2,  i = 0 .. N_TRIANGLES-k-2

for k in range(N_TRIANGLES):
	n_tri  = N_TRIANGLES - k      # triangulos en este nivel
	n_roof = n_tri - 1            # techos entre ellos
	y_tri  = level_y_ctr[k]
	y_roof = level_y_roof[k]
	x_orig = x_start + k * (x_step / 2.0)

	for i in range(n_tri):
		xi = x_orig + i * x_step
		add_card(xi - sep_x, y_tri, 0.0, -TILT, 0.0, -SPHERE_RADIUS)
		add_card(xi + sep_x, y_tri, 0.0,  TILT, 0.0, +SPHERE_RADIUS)

	for i in range(n_roof):
		xi    = x_orig + i * x_step
		x_mid = xi + x_step / 2.0
		add_card(x_mid, y_roof, 0.0, pi / 2.0, 0.0, SPHERE_RADIUS)

# =============================================================================
# CARTA EXTRA: "RUIDO" SOBRE LA PUNTA DE LA PIRAMIDE
# =============================================================================
# Posicion: 2 lados largos por encima del pico de la piramide (nivel N-1).
# El triangulo del ultimo nivel siempre esta centrado en x=0 (por simetria).
# Angulos arbitrarios distintos a los de la piramide para que caiga con
# rotacion y derrumbe todo al chocar.
y_apex  = level_y_roof[N_TRIANGLES - 1]          # superficie de la punta
y_extra = y_apex + 2.0 * CARD_HEIGHT             # 2 lados largos mas arriba

add_card(0.0, y_extra, 0.0, pi / 3.0, pi / 5.0, SPHERE_RADIUS)

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
