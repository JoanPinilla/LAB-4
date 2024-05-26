from cmath import pi
import numpy as np
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

# Autor: Jose Alfaro, Joan Pinilla, Jhonatan Gonzales

# Definición de torques máximos para cada motor
torques = [500, 400, 350, 350, 350]

# Posiciones de referencia para cada caso
positions_deg = [
    [0, 0, 0, 0, 0],
    [25, 25, 20, -20, 0],
    [-35, 35, -30, 30, 0],
    [85, -20, 55, 25, 0],
    [80, -35, 55, -45, 0]
]

positions_analog = [[514,510,818,512,512],[597,597,888,444,512],[393,630,716,616,512],[802,445,1000,599,512],[786,395,1000,360,546]
]

# Función para enviar comandos a los motores Dynamixel
def send_joint_command(command, id_num, addr_name, value, delay):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command, id_num, addr_name, value)
        rospy.sleep(delay)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

# Función de callback para manejar los estados de las articulaciones
def callback(data):
    global current_positions
    current_positions = np.multiply(data.position, 180 / pi)
    current_positions[2] -= 90

# Función para imprimir las posiciones actuales de las articulaciones
def print_positions(real, theoretical):
    print('\nÁngulos motores:\n')
    for i, pos in enumerate(real):
        print(f'{i + 1}: {pos:.2f}°\t', end=' ')
    error_rms = np.sqrt(np.mean(np.subtract(theoretical, real) ** 2))
    print(f'\n\nError RMS: {error_rms:.2f}°\n')

# Función para mover gradualmente las articulaciones hacia una posición objetivo
def move_partial(joint_index, goal_position, current_position):
    N = 5
    delta = (goal_position - current_position) / N
    for i in range(N):
        send_joint_command('', joint_index + 1, 'Goal_Position', int(current_position + delta * (i + 1)), 0.5)

# Función principal
if __name__ == '__main__':
    try:
        # Inicializar el nodo de ROS
        rospy.init_node('joint_listener', anonymous=True)
        rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)

        # Configurar los límites de torque de los motores
        for i, torque in enumerate(torques):
            send_joint_command('', i + 1, 'Torque_Limit', torque, 0)

        # Ir a la posición de home
        print('Ir a la posición de home\n')
        for i, pos in enumerate(positions_analog[0]):
            send_joint_command('', i + 1, 'Goal_Position', pos, 1)
            print(f'Movimiento del eslabón {i + 1}\n')
            time.sleep(0.5)
        print('En la posición de home\n')

        # Imprimir las posiciones reales respecto a la posición de home
        print_positions(current_positions, positions_deg[0])

        # Realizar la rutina de movimiento para el caso seleccionado
        selected_case = int(input('Seleccione el caso a ejecutar (1-4): '))
        print(f'Iniciando rutina para el caso {selected_case}\n')
        for i, pos in enumerate(positions_analog[selected_case]):
            print(f'Movimiento del eslabón {i + 1}')
            move_partial(i, pos, positions_analog[0][i])
        print('Rutina finalizada.')

        # Imprimir las posiciones actuales y teóricas
        print_positions(current_positions, positions_deg[selected_case])
    except rospy.ROSInterruptException:
        pass

