#!/usr/bin/env python3

import rospy
from os import system
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import *
from tf import TransformListener
import numpy as np
import math

#================================ Definições Globais ===================================

# TARGET [X,Y]
targets = np.array([[-0.10, -10.99], [2.31, -6.24], [7.10, -3.87], [3.86, 0.07], [-6.30, 0.74], [-4.26, 6.56], [6.17, 12.14]])

# Distância ate alvo e mínima para chegar ao alvo
distancia =  0.0
distancia_x = 0.0
distancia_y = 0.0
distancia_minima = 0.5

# Configurações do Robo
velocidade = 0.8
velocidade_rotacao = 0.7
theta = 0.0

coordanadas_polares = np.zeros((3,3))

# Posição e Orientação atual do robo
posicao_x = 0.0
posicao_y = 0.0
orientacao_x = 0.0
orientacao_y = 0.0
orientacao_z = 0.0
orientacao_w = 0.0
angulo = 0.0

kp = 3.0
ka = 8.0
kb = -0.1

anteriorB = 0.0
distancia_anterior = 0.0
anteriorVA = 0.0

#================================ Funções =================================================

odometry_msg = Odometry()
velocity = Twist()


def odometry_callback(data):
	global odometry_msg
	odometry_msg = data
	

# Autor: Maria Carolina
# Função: Pegar a posição atual do robo
# Argumentos de Entrada: void
# Retorno: void
# Observação: A função atribui valores para variáveis globais como: posicao_x, posicao_y, 
# 		orientacao_x, orientacao_y, orientacao_z, orientacao_w, angulo
def posicao_atual():
    global posicao_x, posicao_y, orientacao_x, orientacao_y, orientacao_z, orientacao_w, angulo
    
    posicao_x = odometry_msg.pose.pose.position.x
    posicao_y = odometry_msg.pose.pose.position.y

    orientacao_x = odometry_msg.pose.pose.orientation.x
    orientacao_y = odometry_msg.pose.pose.orientation.y
    orientacao_z = odometry_msg.pose.pose.orientation.z
    orientacao_w = odometry_msg.pose.pose.orientation.w

    (roll, pitch, yaw) = euler_from_quaternion ([orientacao_x, orientacao_y, orientacao_z, orientacao_w])
    angulo = yaw
	

# Autor: Waleson Melo
# Função: Parar o robo
# Argumentos de Entrada: void
# Retorno: void
# Observação: A função atribui 0.0 para a velocidade linear e angular do robo
def move_frente():
	velocity.linear.x = velocidade


# Autor: Waleson Melo
# Função: Dobra o robo
# Argumentos de Entrada: rotacao: Valor que define se a rotação é positiva ou negativa
# Retorno: void
# Observação: A função atribui qualquer valor passado pelo parametro de entrada para a velocidade angular do robo
def dobra(rotacao):
	velocity.angular.z = rotacao


# Autor: Maria Carolina
# Função: Parar o robo
# Argumentos de Entrada: void
# Retorno: void
# Observação: A função atribui 0.0 para a velocidade linear e angular do robo
def para():
	velocity.linear.x = 0.0
	velocity.angular.z = 0.0
	pub.publish(velocity)


# Autor: Maria Carolina
# Função: Calcular a distancia entre o Robo e o Alvo
# Argumentos de Entrada: alvo_x: Posição no eixo x do alvo, alvo_y: Posição no eixo y do alvo,
# 		robo_x: Posição no eixo x do robo, robo_y: Posição no eixo y do robo
# Retorno: void
# Observação: A função atribui valores para variáveis globais como: distancia_x, distancia_y e distancia
def distancia_ate_alvo(alvo_x, alvo_y, robo_x, robo_y):
	global distancia_x, distancia_y, distancia
	distancia_x = alvo_x - robo_x
	distancia_y = alvo_y - robo_y
	
	distancia = math.sqrt((distancia_x)**2 + (distancia_y)**2)


# Autor: Maria Carolina, Waleson Melo
# Função: Realiza os calculos necessários para que o robo siga até o alvo
# Argumentos de Entrada: void
# Retorno: void
# Observação: A função atribui valores para variáveis globais como: theta, coordanadas_polares,
# 		distancia_anterior, anteriorVA, anteriorB. 
def caminho_alvo():
	global theta, coordanadas_polares, distancia_anterior, anteriorVA, anteriorB
	
	alpha = -theta + math.atan2(distancia_x - 0.05, distancia_y - 0.05)
	beta = -theta - alpha

	coordanadas_polares[0][0] = (-kp * distancia * math.cos(alpha))
	coordanadas_polares[0][0] = coordanadas_polares[0][0] + (coordanadas_polares[0][0] - distancia_anterior)
	pa = (kp * math.sin(alpha) - ka * alpha - kb * beta)
	pb = (-kp * math.sin(alpha))

	if( pb <= (anteriorB ) ):
		coordanadas_polares[1][0] = (-pa) 
		coordanadas_polares[2][0] = (-pb)
	else:
		coordanadas_polares[1][0] = (pa)
		coordanadas_polares[2][0] = pb
	
	v = kp * coordanadas_polares[0][0]
	va = (ka * coordanadas_polares[1][0] + kb * coordanadas_polares[2][0])

	alpha_graus = math.degrees(alpha)
	
	move_frente()

	if(va < 0 and (alpha_graus > 0 and alpha_graus < 90)):
		dobra(velocidade_rotacao)
		print("QIE--->rolagem esq")
	elif va > 0 and (alpha_graus > 0 and alpha_graus < 90):
		dobra(-velocidade_rotacao)
		print("QIE--->rolagem dir")
	else:
		if va < 0 and (90 > alpha < 180):
			dobra(-velocidade_rotacao)
			print("QID--->rolagem esq")
		elif va > 0 and (90 > alpha < 180):
			dobra(velocidade_rotacao)
			print("QID--->rolagem dir")
	
	if va < 0 and (0 > alpha_graus < -90):
		dobra(velocidade_rotacao)
		print("QSD--->rolagem dir")
	elif va > 0 and (0 > alpha_graus < -90):
		dobra(-velocidade_rotacao)
		print("QSD--->rolagem esq")

	distancia_anterior = distancia
	anteriorB = pb
	anteriorVA=-va
	pub.publish(velocity)


#================================ Função Principal ========================================

# Autor: Maria Carolina, Waleson Melo
# Execução principal do programa
if __name__ == "__main__":
	# Node
	rospy.init_node("trabalho", anonymous=False)  

	# Subscribers
	rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)

	# Publishers
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	pub_pos = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
  
	rate = rospy.Rate(10) #10hz
	count = 0
	continua = True

	while not rospy.is_shutdown() and continua:
		posicao_atual()
		distancia_ate_alvo(targets[count][0], targets[count][1], posicao_x, posicao_y)
		caminho_alvo()

		rospy.loginfo("T: (%.2f, %.2f)" % (targets[count][0], targets[count][1]))
		rospy.loginfo("P: (%.2f, %.2f)" % (posicao_x, posicao_y)) 
		rospy.loginfo("D: %.2f" % distancia)

		if(distancia < distancia_minima):
			count += 1
			if(count >= len(targets)):
				continua = False

		rate.sleep()

	para()
