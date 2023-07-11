import vrep
import numpy as np
import matplotlib.pyplot as plt
import math
import time

RAD2DEG = math.pi / 180  

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

def set_joint_positions(clientID, joint_handles, joint_positions):
    for i, handle in enumerate(joint_handles):
        vrep.simxSetJointTargetPosition(clientID, handle, joint_positions[i], vrep.simx_opmode_oneshot)

def linear_blend(t):
    if t < 0 or t > 1:
        return 0
    return 3 * t**2 - 2 * t**3

def parabolic_blend(t):
    if t < 0 or t > 1:
        return 0
    return 1 - (1 - t)**2

def generate_trajectory(q0, qf, v_max, a_max, dt):
    num_joints = len(q0)
    num_steps = int(np.ceil(np.max(np.abs(qf - q0)) / v_max / dt))
    trajectory = np.zeros((num_joints, num_steps))

    for i in range(num_joints):
        q_start = q0[i]
        q_end = qf[i]
        delta_q = q_end - q_start

        for j in range(num_steps):
            t = j / num_steps
            blend = linear_blend(t)
            if j == 0:
                blend = parabolic_blend(t)
            elif j == num_steps - 1:
                blend = parabolic_blend(1 - t)

            trajectory[i, j] = q_start + delta_q * blend

    return trajectory

def plot_trajectory(trajectory, dt):
    num_joints, num_steps = trajectory.shape
    t = np.arange(0, num_steps * dt, dt)

    fig, axs = plt.subplots(num_joints, 1, sharex=True)
    for i in range(num_joints):
        axs[i].plot(t, trajectory[i])
        axs[i].set_ylabel(f'Joint {i+1} Position')
        axs[i].grid(True)
    axs[-1].set_xlabel('Time (s)')

    plt.tight_layout()
    plt.show()


def end_effector(clientID):
    # Define tamaho do passo e o timeout
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
    time.sleep(0.5)

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



def main():
    clientID = connect_to_simulator()

    # Obter handles das juntas do robô UR5
    joint_handles = []
    for i in range(1, 7):
        joint_name = 'UR5_joint' + str(i)
        _, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
        joint_handles.append(handle)

    # Definir a posição inicial e final das juntas
    q0 = np.array([0, 0, 0, 0, 0, 0])  # Posição inicial das juntas (rad)
    qf = np.array([0, 22.5 * RAD2DEG, 67.5 * RAD2DEG, 0, -90 * RAD2DEG, 0])
    # Definir os limites de velocidade e aceleração
    v_max = np.radians(60)  # Velocidade máxima das juntas (rad/s)
    a_max = np.radians(120)  # Aceleração máxima das juntas (rad/s^2)

    # Definir o intervalo de tempo
    dt = 0.01  # Intervalo de tempo entre os pontos (s)

    # Gerar a trajetória
    trajectory = generate_trajectory(q0, qf, v_max, a_max, dt)

    # Enviar a trajetória para o robô
    for i in range(trajectory.shape[1]):
        set_joint_positions(clientID, joint_handles, trajectory[:, i])
        vrep.simxSynchronousTrigger(clientID)  # Dispara a próxima etapa da simulação
        vrep.simxGetPingTime(clientID)  # Espera até a simulação ser concluída
    exit()    
    #end_effector(clientID)
    
    # Exibir a trajetória
    plot_trajectory(trajectory, dt)

    disconnect_from_simulator(clientID)

if __name__ == '__main__':
    vrep.simxFinish(-1)  # Fecha todas as conexões existentes
    main()
