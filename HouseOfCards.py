#Script para simular un granular almacenado en un contenedor como un silo
#************************************************************************
#Damyan Benjamyn Santander Huerta
#Lunes 10 de Abril 2023
#**********************
from __future__ import print_function
from yade import plot, polyhedra_utils
from yade import qt
import numpy as npy
from numpy import pi
import random
#************
#Materiales a usar
#*****************
#acero para las paredes y el piso
m_pared = PolyhedraMat()
m_pared.density = 8000  #kg/m^3
m_pared.young = 20.6E10  #[Pa]
m_pared.poisson = 0.5
m_pared.frictionAngle = 0.6  #rad
#acero temporalmente para las particulas
m_grano = PolyhedraMat()
m_grano.density = 1130 #kg/m^3
m_grano.young = 4e6  #[Pa]
m_grano.poisson =.5
m_grano.frictionAngle = 0.6  #rad
#********************************
#crear las paredes
#dimensiones
px = 15e-2 #[m]
py = 10e-2 #[m] 40 cm en el montaje, alto de la pared, aunque el muro es infinito hacia arriba
#pared del piso
O.bodies.append(utils.wall(position = (0,0,0), axis=1, sense=1, material=m_pared,color = (0,0.8,1)))
#la posicion es la constante del vector posicion
#axis es el eje en el que esta orientado
#sense es la direccion en que empuja los granos, en este caso
#los devuelve hacia adentro, y se le asigna su Material
#******************************************************
#funcion que genera un grano por extrusion de un poligono regular
#parametros
#(xc,yc) posicion del centro del granos
#Radius es el radio del poligonos
#thickness es la altura de la extrusion
#resolution es el numero de puntas del poligono
#angle es el angulo total a recorrer, ideal 2*pi
#giro es el angulo de giro del poligono
def custom_polyhedron(xc,yc,a,b,thickness,resolution,angle,giro):
	f = angle/resolution
	if angle < 2*pi:
		resolution = resolution+1
	x = a*cos(giro)+xc
	y = b*sin(giro)+yc
	faces = ((x,y,0),(x,y,thickness))
	for i in npy.arange(1,5):
		if i == 1:
			x = xc
			y = b+yc
		if i == 2:
			x = a+xc
			y = b+yc
		if i == 3:
			x = a+xc
			y = yc
		if i == 4:
			x = xc
			y = yc
		new_face=((x,y,0),(x,y,thickness))
		faces = faces+new_face
	poli = polyhedra_utils.polyhedra(m_grano,v =  faces)
	poli.state.blockedDOFs='xYZ'
	poli.shape.color = (1,0.6,0.1)
	poli.shape.wire = False
	#nose puede mover en z ni girar en X ni en Y
	return poli
#**************
#generar granos
h = -0.07E-2 #[m] altura poligono
R = 0.2E-2 #[m] radio circulo circunscrito
angle = 2*pi # angulo final
b = 9e-2 #[m]
a = 6e-2 #[m]
Res =  4
xc = 0
yc = 1e-3
n_c = 2 #numero de cartas
for i in range(0,2*n_c):
	t = custom_polyhedron(xc,yc,a,b,h,Res,angle,0)
	t.state.ori = ((0,1,0),pi/2)
	t.state.pos = (xc,yc+b/2, (i)*b*npy.cos(pi/6)*0.6)
	if i+2 & 1 == 0:
		t.state.ori = ((1,0,0),pi/6)
	else:
		t.state.ori = ((1,0,0),-pi/6)
	O.bodies.append(t)


#interacciones y fisicas
O.engines = [
        ForceResetter(),
        InsertionSortCollider([Bo1_Polyhedra_Aabb(), Bo1_Wall_Aabb(), Bo1_Facet_Aabb()]),
        InteractionLoop(
                [Ig2_Wall_Polyhedra_PolyhedraGeom(),
                 Ig2_Polyhedra_Polyhedra_PolyhedraGeom(),
                 Ig2_Facet_Polyhedra_PolyhedraGeom()],
                [Ip2_PolyhedraMat_PolyhedraMat_PolyhedraPhys()],  # collision "physics"
                [Law2_PolyhedraGeom_PolyhedraPhys_Volumetric()]  # contact law -- apply forces
        ),
        NewtonIntegrator(damping=0.4, gravity=(0, -9.81, 0)), #gravedad
        PyRunner(command='checkUnbalanced()', realPeriod=2),
]
O.dt=polyhedra_utils.PWaveTimeStep()*0.05
O.trackEnergy = True
# if the unbalanced forces goes below .05, the packing
# is considered stabilized, therefore we stop collected
# data history and stop
def checkUnbalanced():
	if unbalancedForce() < .05:#0.05
		O.pause()
		plot.saveDataTxt('bbb.txt.bz2')
		# plot.saveGnuplot('bbb') is also possible
V = qt.View()
O.saveTmp()
