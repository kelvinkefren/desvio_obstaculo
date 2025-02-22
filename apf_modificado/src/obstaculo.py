#!/usr/bin/env python3

import rospy
import random
import subprocess
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
import numpy as np
from gazebo_msgs.srv import SpawnModel,SetModelState
import math
import tf
from gazebo_msgs.msg import ModelState
import roslaunch
import os
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import cmath


def save_robot_position(robot_positions,x, y):
    robot_positions.append((x, y))
    
    return robot_positions
 

def plot_robot_positions(robot_positions, obstacle_position, obstacle_radius):
    x = [pos[0] for pos in robot_positions]
    y = [pos[1] for pos in robot_positions]
    x_box = [pos[0] for pos in obstacle_position]
    y_box = [pos[1] for pos in obstacle_position]
    fig, ax = plt.subplots()
    ax.plot(x, y, 'r',x_box,y_box,'b')
    
    # plot obstacle as circle
    #obstacle_circle = Circle(obstacle_position, obstacle_radius, fill=False, edgecolor='b', linewidth=2)
    #ax.add_artist(obstacle_circle)
    # set axis limits
    ax.set_xlim([-2, 10])
    ax.set_ylim([-3, 10])
    
    plt.show()


def adjust_angle(angle):
    angle = np.mod(angle, 2 * np.pi)
    if angle > np.pi:
        angle = angle - 2 * np.pi
    if angle < -np.pi:
        angle = angle + 2 * np.pi
    angulo_ajustado = angle
    return angulo_ajustado
    
def spawn_husky_launch(husky_name, position_x=9.0, position_y=9.0, yaw=0.0):
    command = "roslaunch husky_gazebo spawn_husky.launch name:={0} x:={1} y:={2} yaw:={3}".format(husky_name, position_x, position_y, yaw)
    subprocess.call(command, shell=True)

def set_husky_velocity(husky_name, linear_x):
    pub = rospy.Publisher("/{0}/cmd_vel".format(husky_name), Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = linear_x
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()
 
    
def modified_attractive_force(pos, pg, epsilon,dg,Nog):
    #calculates the attractive force between a target position "pg" and a current position "pos". The function first calculates the direction 	vector "nog" between the two positions   
    
    #calculate the Atractive force 
    Fatt = epsilon * dg *Nog
    
    return Fatt
 
def calculate_Fre(d, tau, Dm, dg, Ne, Rts, Not, Vto, tetha, Nog,Notperp):
	var1 = (1./(d-tau)-1/Dm)
	var2 = (dg**2)/((d-tau)**2)
	Fre1 = -2 * Ne * Rts * var1 * var2 * Not
	Fre2 = 2 * Ne * Rts * dg / d * np.linalg.norm(Vto)**2 * (np.cos(tetha) * np.sin(tetha)) * Notperp
	Fre3 = 2 * Ne * Rts * dg * (var1**2 + np.linalg.norm(Vto)**2 * np.cos(tetha)**2) * Nog
	Fre = Fre1 + Fre2 + Fre3
	return Fre
       
def calculate_Frs(d, tau, dg, phou0, Ns, Rts, Not, Nog):
 	var1 = 1/(d-tau) - 1/phou0
 	var2 = (dg**2)/(d**2)
 	Frs1 = -Ns * Rts * var1 * var2 * Not
 	Frs3 = Ns * Rts * dg * var1**2 * Nog
 	return Frs1 + Frs3  
    
def calculate_Frd(d,Dm,phou0,tetha,Vto,theta_m_degree,AngleDiff,Pot,Nd,Rts,dg,Not,Notperp,Nog):
	var1 = ((1./(d-Dm))-(1/phou0))
	var2 = (Dm/(d*np.sqrt(d**2-Dm**2)))
	var3 = (np.sin(np.radians(tetha))/np.linalg.norm(Vto))
	var4 = (np.sin(np.radians(theta_m_degree))/np.linalg.norm(Vto))
	var5 = ((np.exp(AngleDiff)-1)/((d-Dm)**2))
	Fto0 = var1*(var2 + var4)
	var6 = (1/np.linalg.norm(Pot)+np.cos(np.radians(tetha))/np.linalg.norm(Vto))
	var7 = np.linalg.norm(Vto)*(np.exp(AngleDiff)-1)/(d*(d-Dm)**2)
	Ftop = var1*(1/np.linalg.norm(Pot)+np.cos(np.radians(theta_m_degree))/np.linalg.norm(Vto)) 
	Frd1 = -Nd*Rts*dg**2*(var1*np.exp(AngleDiff)*(var2 + var3)+var5-Fto0)*Not
	Frd3 = Nd*Rts*dg**2*var1*(np.exp(AngleDiff)-1)*Nog 
	Frd2 = -Nd*Rts*dg**2*(var1*np.exp(AngleDiff)*var6*var7-Ftop)*Notperp
	Frd = Frd1 + Frd2 + Frd3
	return Frd

def modified_potential_field(goal,obstacles,obstacles_radius,obstacles_velocities,current_position,epsilon,Nd,Ns,Ne,tau,Ros,Dsafe,phou0,vos_velocity):
	#vector that point from the robot_position to the goal
	pg = goal - current_position 		
	#print(obstacles)
	#print(obstacles_radius)
	#print(obstacles_velocities)
	#teste
	count = 0
	#the vector's size is the distance until the goal.
	dg = np.linalg.norm(pg) 
	
	#Unit vector in goal direction
	Nog = pg/dg  
	
	#initiate Fatt
	Fatt = np.zeros_like(current_position)
	# Initialize the repulsive force
	Frep = np.zeros_like(current_position)
	
	#calculate the attractive force	
	Fatt = modified_attractive_force(current_position, pg, epsilon,dg,Nog)
	#print("goal = ",goal," posicao atual = ",current_position," distancia = ",dg,"vetor normal =",Nog)
	#print(" posicao atual = ",current_position,"goal = ",goal,"Fatt = ",Fatt," distancia = ",dg)
    	# Calculate the repulsive force from each obstacle
    	
	for i,obstacle in enumerate(obstacles):
		#initiate Frd,Frs,Fre
		Frd = np.zeros_like(current_position)
		Fre = np.zeros_like(current_position)
		Frs = np.zeros_like(current_position)
		
		
		#print("contagem = ",count+1," i e seus obstacles",i,obstacles[i],obstacles_velocities[i],obstacles_radius[i])
		#print("obstaculo = ",obstacle)
		#get some parameters related to the obstacle i
		#verificar angulacao entre Vts e Nog. Caso a ângulação for menor que 5 graus, aplicar uma força perpendicular a Vts para a direita.
		
		
		d, Dm, CR, Pot, theta_m_radian, Vto, tetha, Not, AngleDiff, Notperp, Rts = iniciation(obstacles[i], current_position, vos_velocity, obstacles_velocities[i], obstacles_radius[i], Dsafe, phou0, Ros,Nog)
		
			
				
		#print ("d = ",d," CR =",CR," theta =",tetha,"theta_m_radian",theta_m_radian," Dm =",Dm)
		if d <= CR and tetha < theta_m_radian and Dm <= d:
    			if not np.array_equal(obstacles_velocities[i], np.array([0, 0])):
    				#verificar angulacao entre Vts e Nog. Caso a ângulação for menor que 5 graus, aplicar uma força perpendicular a Vts para a direita.
    				#angulacao = np.degrees(np.arccos(np.dot(Pot, Nog)/(np.linalg.norm(Pot) * np.linalg.norm(Nog))))
    				#if angulacao > 5 and angulacao > 355:
    				#	Notperp = np.array([-Not[1], Not[0]])
    				Frd = calculate_Frd(d,Dm,phou0,tetha,Vto,theta_m_radian,AngleDiff,Pot,Nd,Rts,dg,Not,Notperp,Nog)
    				print("d =",d,"< CR =",CR," dinamic --------------  Frd = ",Frd)
    			else:
        			Frs = calculate_Frs(d, tau, dg, phou0, Ns, Rts, Not, Nog)
        			print("d =",d,"< CR =",CR," static --------------  Frs = ",Frs)
		else:
    			if d < Dm :
        			#print(" d =",d, " tau =",tau, " Dm =",Dm, " dg =",dg, " Ne =",Ne, " Rts =",Rts," Not =", Not, " Vto =",Vto, " tetha =",tetha, " Nog =",Nog," Notperp =",Notperp)
        			Fre = calculate_Fre(d, tau, Dm, dg, Ne, Rts, Not, Vto, tetha, Nog,Notperp)
        			print("d =",d,"< Dm =", Dm," --------------  Fre = ",Fre)
    			else:
        			print(" ")
		
		
		Frep += Frd + Frs + Fre
		print("obstaculo numero: ",i,"definicao do obst =",obstacles[i],"velocidade do obstaculo",obstacles_velocities[i],"Frep = ",Frep,"tetha = ",tetha,"theta_m = ",theta_m_radian)
		#Frep += Frd
    	# Return the sum of the attractive and repulsive forces as the total force
	total_force = Fatt + Frep
	print("FORCA TOTAL = ",total_force)
	return total_force


def iniciation(obstacles, current_position, vos_velocity, obstacles_velocities, obstacles_radius, Dsafe, phou0, Ros,Nog):
    #d: Euclidean distance between the current position of the boat and the i-th obstacle.
    d = np.linalg.norm(obstacles - np.array(current_position))
    
    
    #Dm: The minimum safe distance between the boat and the i-th obstacle. It is calculated as the sum of the radius of the obstacle, the safety margin and the safety distance 
    Dm = Ros + Dsafe + obstacles_radius
    print("Ros = ",Ros,"Dsafe =", Dsafe,"obstacles_radius = ",obstacles_radius)
    #CR: The critical distance from the i-th obstacle. It is calculated as the sum of Dm and phou0.
    CR = Dm + phou0
    
    #Pot: The vector pointing from the current position of the boat to the i-th obstacle.
    Pot = obstacles - current_position 
    
    #theta_m_radian: The angle between the vector Pot and a vector pointing to the direction of the normal line passing through the point on the circumference of the obstacle at a safe distance from it.
    #print("d = ",d," Dm = ",Dm)
    if d >= Dm:
    	theta_m_radian2 = np.arctan2(Dm, np.sqrt(d**2 - Dm**2))
    	theta_m_radian = np.degrees(theta_m_radian2)
    	#theta_m_radian = adjust_angle(theta_m_radian)
    else:
        theta_m_radian = 0
    #theta_m_radian = np.arctan2(Dm,cmath.sqrt(d**2 - Dm**2))
    #print("theta_m =",math.degrees(theta_m_radian))
    #vto: The relative velocity between the boat and the i-th obstacle. 
    Vto = vos_velocity - obstacles_velocities
    print("VOS = ",vos_velocity,"VTs = ",obstacles_velocities)
    
    #tetha: The angle between the vector Pot and the relative velocity vector vto.
    extra = 1e-8  # You can adjust this value as needed
    tetha2 = np.arccos(np.dot(Pot, Vto) / (np.linalg.norm(Pot) * np.linalg.norm(Vto) + extra))
    tetha = np.degrees(tetha2)
    #tetha = adjust_angle(tetha)
    #print("angulo = ",math.degrees(tetha),"angulo_obst = ",math.degrees(theta_m_radian) )
    
    #Not is a unit vector pointing from the current position to the obstacle
    Not = (obstacles - current_position) / d
    
    AngleDiff = theta_m_radian-tetha 
    angulacao = np.degrees(np.arccos(np.dot(Not, Nog)/(np.linalg.norm(Not) * np.linalg.norm(Nog))))
    histerese = 0
    if angulacao < 5 or angulacao > 355:
    	histerese = 0.1
    Notperp = comparar_vetores(Pot, Vto, obstacles_velocities,Not,histerese)
    Notperp = decide_rotacao(vos_velocity,obstacles_velocities,Not)
    #Notperp = determine_side(Pot, Vto,Not)
    Rts = obstacles_radius
    
    return d, Dm, CR, Pot, theta_m_radian, Vto, tetha, Not, AngleDiff, Notperp, Rts

def comparar_vetores(v1, v2, v3,Not,histerese): #Pot, Vto, obstacles_velocities,Not,histerese
    
    # calcula o produto vetorial entre os vetores v1 e v2, e entre os vetores v3 e v2
    produto_v1_v2 = v1[0]*v2[1] - v1[1]*v2[0]
    produto_v3_v2 = v3[0]*v2[1] - v3[1]*v2[0]

    # verifica a posição de v1 em relação a v2
    if produto_v1_v2 > histerese:  # v1 está à esquerda de v2
        # verifica a posição de v3 em relação a v2
        if produto_v3_v2 > histerese:  # v3 está à direita de v2
            return np.array([Not[1], -Not[0]])  # Rotate counterclockwise#"obstáculo à esquerda, mas andando para direita"
        elif produto_v3_v2 <= -histerese:  # v3 está à esquerda de v2
            return np.array([-Not[1], Not[0]])  # Rotate clockwise#"obstáculo à esquerda e obstáculo indo para a esquerda também"
    elif produto_v1_v2 < -histerese:  # v1 está à direita de v2
        # verifica a posição de v3 em relação a v2
        if produto_v3_v2 <= histerese:  # v3 está à esquerda de v2
            return np.array([-Not[1], Not[0]])  # Rotate clockwise#"obstáculo à direita, mas andando para esquerda"
        elif produto_v3_v2 > -histerese:  # v3 está à direita de v2
            return np.array([Not[1], -Not[0]])  # Rotate counterclockwise#"obstáculo à direita e andando para direita"

def decide_rotacao(vr, vo,Not):
    """
    Decide a direção da rotação para desviar do obstáculo.

    :param vr: tuple (vrx, vry), vetor velocidade do robô
    :param vo: tuple (vox, voy), vetor velocidade do obstáculo
    :return: string, "anti-horário", "horário" ou "paralelo"
    """
    vrx, vry = vr
    vox, voy = vo

    # Calcula a componente z do produto vetorial
    pz = vrx * voy - vry * vox
    
    # Calcula o produto escalar e as magnitudes dos vetores de velocidade
    dot_product = vrx * vox + vry * voy
    vr_magnitude = math.sqrt(vrx ** 2 + vry ** 2)
    vo_magnitude = math.sqrt(vox ** 2 + voy ** 2)

    # Calcula o ângulo entre os vetores de velocidade em graus
    angle_rad = math.acos(dot_product / (vr_magnitude * vo_magnitude))
    angle_deg = math.degrees(angle_rad)

    # Se o ângulo estiver entre 175 e 185 graus, escolha uma direção de rotação padrão (anti-horário, por exemplo)
    if 165 <= angle_deg <= 195:
        return -np.array([Not[1], -Not[0]])  #"anti-horário"
    elif pz > 0:
        return -np.array([Not[1], -Not[0]])  #"anti-horário"
    elif pz < 0:
        return -np.array([-Not[1], Not[0]]) #"horário"
    else:
        return -np.array([Not[1], -Not[0]]) #"paralelo"  


    # caso não tenha retornado ainda, não há comparação possível
    #print("Não há comparação possível")
    return np.array([-Not[1], Not[0]])
    
def determine_side(v1, v2, Not):
    # Calculate the unit vector pointing along v1
    v1_norm = np.linalg.norm(v1)
    v1_unit = v1 / v1_norm if v1_norm > 0 else np.array([0, 0])
    
    # Calculate the unit vector pointing along v2
    v2_norm = np.linalg.norm(v2)
    v2_unit = v2 / v2_norm if v2_norm > 0 else np.array([0, 0])
    
    # Calculate the dot product of the two unit vectors
    dot_product = np.dot(v1_unit, v2_unit)
    
    # Determine the orientation of v2 relative to v1
    if dot_product > 0:
        # v2 is to the right of v1 or collinear with it
        return np.array([-Not[1], Not[0]])  # Rotate clockwise
    else:
        # v2 is to the left of v1
        return np.array([Not[1], -Not[0]])  # Rotate counterclockwise
        

def force_to_velocities(total_force, heading):
    # Calculate the linear velocity as the magnitude of the total force
    linear_velocity = np.linalg.norm(total_force)
    
    # Calculate the angular velocity as the angle between the heading and the direction of the total force
    angular_velocity = (np.arctan2(total_force[1], total_force[0]) - (heading))
    #angular_velocity = np.arctan2(total_force[1].real, total_force[0].real) - heading
    
    return linear_velocity, angular_velocity
    
def change_position(name,pos):
	rospy.wait_for_service('/gazebo/set_model_state')
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	model_state = ModelState()
	model_state.model_name = name
	model_state.pose.position.x = pos[0]
	model_state.pose.position.y = pos[1]
	model_state.pose.orientation = Quaternion(0, 0, math.radians(40), 1)
	
	set_model_state(model_state)

def change_position_velocity(name,pos,vel):
	rospy.wait_for_service('/gazebo/set_model_state')
	set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
	model_state = ModelState()
	model_state.model_name = name
	model_state.pose.position.x = pos[0]
	model_state.pose.position.y = pos[1]
	model_state.twist.linear.x = vel[0]
	model_state.twist.linear.y = vel[1]
	model_state.pose.orientation = Quaternion(0, 0, 0, 1)
	
	set_model_state(model_state)
def spawn_obstacle():
    package = 'gazebo_ros'
    executable = 'spawn_model'
    node = roslaunch.core.Node(package, executable)
    node.args = "-urdf -param /husky_description -model obst_1 -x 1 -y 1"

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)
    rospy.loginfo("Spawned Husky robot with name 'obst_1' and position (1, 1)")
    

    

