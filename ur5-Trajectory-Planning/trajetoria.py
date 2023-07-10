# Esse código usa o método de trajetória cúbica para mover o braço robótico até a posição onde ele irá mover o end-effector para o furo.

import vrep
import numpy as np
from scipy.interpolate import interp1d
import time
import matplotlib.pyplot as plt
import math

RAD2DEG = math.pi / 180  

def plot_trajectory(trajectory):
    plt.figure()
    plt.plot(trajectory[:, 0], label='Joint 1')
    plt.plot(trajectory[:, 1], label='Joint 2')
    plt.plot(trajectory[:, 2], label='Joint 3')
    plt.plot(trajectory[:, 3], label='Joint 4')
    plt.plot(trajectory[:, 4], label='Joint 5')
    plt.plot(trajectory[:, 5], label='Joint 6')
    plt.xlabel('Pontos da trajetória')
    plt.ylabel('Ângulo das juntas (rad)')
    plt.title('Trajetória do Robô UR5')
    plt.legend()
    plt.show()
    
def connect_to_simulator():
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conecta ao CoppeliaSim
    if clientID != -1:
        print('Conexão estabelecida com sucesso!')
    else:
        print('Falha ao conectar ao CoppeliaSim.')
    return clientID

def disconnect_from_simulator(clientID):
    vrep.simxFinish(clientID)
    print('Conexão encerrada.')

def get_object_handle(clientID, object_name):
    _, handle = vrep.simxGetObjectHandle(clientID, object_name, vrep.simx_opmode_blocking)
    return handle

def set_joint_positions(clientID, joint_handles, joint_positions):
    for i, handle in enumerate(joint_handles):
        vrep.simxSetJointTargetPosition(clientID, handle, joint_positions[i], vrep.simx_opmode_oneshot)

def interpolate_trajectory(trajectory, num_points):
    t = np.linspace(0, 1, len(trajectory))
    f = interp1d(t, trajectory, axis=0, kind='cubic')
    t_new = np.linspace(0, 1, num_points)
    return f(t_new)

def execute_trajectory(clientID, joint_handles, trajectory):
    for point in trajectory:
        set_joint_positions(clientID, joint_handles, point)
        vrep.simxSynchronousTrigger(clientID)  # Dispara a próxima etapa da simulação
        vrep.simxGetPingTime(clientID)  # Espera até a simulação ser concluída
        
def end_effector(clientID):
    # Define tamaho do passo e o timetou
    step = 0.005  
    TIMEOUT = 5000
    
    # Define parametros das juntas
    jointNum = 6
    jointName = 'UR5_joint'
    
    # Começa a simular a aplicação do end-effector
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_blocking)
    print("Aplicação do end-effector no furo")
    
    jointHandle = np.zeros((jointNum,), dtype=int)
    for i in range(jointNum):
        errorCode, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i + 1), vrep.simx_opmode_blocking)
        jointHandle[i] = returnHandle
        time.sleep(1)
    
    errorCode, holeHandle = vrep.simxGetObjectHandle(clientID, 'Hole', vrep.simx_opmode_blocking)
    errorCode, ikTipHandle = vrep.simxGetObjectHandle(clientID, 'UR5_ikTip', vrep.simx_opmode_blocking)
    errorCode, connectionHandle = vrep.simxGetObjectHandle(clientID, 'UR5_connection', vrep.simx_opmode_blocking)
    errorCode, targetPosition = vrep.simxGetObjectPosition(clientID, holeHandle, -1, vrep.simx_opmode_streaming)
    time.sleep(0.5)
    errorCode, targetPosition = vrep.simxGetObjectPosition(clientID, holeHandle, -1, vrep.simx_opmode_buffer)
    print('Posição disponível!')
    
    # Ato 1
    
    initConfig = [0, 22.5 * RAD2DEG, 67.5 * RAD2DEG, 0, -90 * RAD2DEG, 0]

    vrep.simxPauseCommunication(clientID, True)
    for i in range(jointNum):
        vrep.simxSetJointTargetPosition(clientID, jointHandle[i], initConfig[i], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    
    vrep.simxGetPingTime(clientID)
    time.sleep(1)

    # Pegue o quartetion
    errorCode, tipQuat = vrep.simxGetObjectQuaternion(clientID, ikTipHandle, -1, vrep.simx_opmode_blocking)

    # Ato 2
    
    targetPosition[2] = targetPosition[2] + 0.15

    # Envie o sinal para o movimento
    vrep.simxPauseCommunication(clientID, 1)
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 21, vrep.simx_opmode_oneshot)
    for i in range(1, 4):
        vrep.simxSetFloatSignal(clientID, 'ICECUBE_' + str(i), targetPosition[i - 1], vrep.simx_opmode_oneshot)
    for i in range(4, 8):
        vrep.simxSetFloatSignal(clientID, 'ICECUBE_' + str(i), tipQuat[i - 4], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, 0)

    # Delay
    j = 0
    signal = 99
    while j <= TIMEOUT and signal != 0:
        j = j + 1
        errorCode, signal = vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking)
        time.sleep(step)

    errorCode = vrep.simxSetIntegerParameter(clientID, vrep.sim_intparam_current_page, 1, vrep.simx_opmode_blocking)

    # Ato 3 posicionsamento do alvo

    targetPosition[2] = targetPosition[2] - 0.05
    time.sleep(2)

    # Envie o sinal do movimento
    vrep.simxPauseCommunication(clientID, 1)
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 21, vrep.simx_opmode_oneshot)
    for i in range(1, 4):
        vrep.simxSetFloatSignal(clientID, 'ICECUBE_' + str(i), targetPosition[i - 1], vrep.simx_opmode_oneshot)
    for i in range(4, 8):
        vrep.simxSetFloatSignal(clientID, 'ICECUBE_' + str(i), tipQuat[i - 4], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, 0)

    # Delay
    j = 0
    signal = 99
    while j <= TIMEOUT and signal != 0:
        j = j + 1
        errorCode, signal = vrep.simxGetIntegerSignal(clientID, 'ICECUBE_0', vrep.simx_opmode_blocking)
        time.sleep(step)
    time.sleep(1)

    vrep.simxSetIntegerParameter(clientID, vrep.sim_intparam_current_page, 0, vrep.simx_opmode_blocking)
    time.sleep(2)
    
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
    vrep.simxSetIntegerSignal(clientID, 'ICECUBE_0', 1, vrep.simx_opmode_blocking)
    time.sleep(0.5)    

def move_to_home_position(clientID, joint_handles):
    home_positions = [0, 0, 0, 0, 0, 0]  # Posição "home" desejada para cada junta
    set_joint_positions(clientID, joint_handles, home_positions)
    vrep.simxSynchronousTrigger(clientID)  # Dispara a próxima etapa da simulação
    vrep.simxGetPingTime(clientID)  # Espera até a simulação ser concluída


def main():
    clientID = connect_to_simulator()

    # Obter handles das juntas do robô UR5
    joint_handles = []
    for i in range(1, 7):
        joint_name = 'UR5_joint' + str(i)
        joint_handle = get_object_handle(clientID, joint_name)
        joint_handles.append(joint_handle)
    
    # Envia o robo para posição home    
    move_to_home_position(clientID, joint_handles)    

    # Definir uma trajetória desejada
    start_position = [0, 0, 0, 0, 0, 0]
    end_position = [0, 22.5 * RAD2DEG, 67.5 * RAD2DEG, 0, -90 * RAD2DEG, 0]
    num_points = 50
    trajectory = np.linspace(start_position, end_position, num_points)

    # Interpolar a trajetória
    interpolated_trajectory = interpolate_trajectory(trajectory, num_points)
    
    # Executar a trajetória
    print("Execução da trajetória")
    execute_trajectory(clientID, joint_handles, interpolated_trajectory)
    
    # Mover o end-effector para o furo
    end_effector(clientID)
    
    disconnect_from_simulator(clientID)
    
    plot_trajectory(interpolated_trajectory)

if __name__ == '__main__':
    vrep.simxFinish(-1)  # Fecha todas as conexões existentes
    main()

