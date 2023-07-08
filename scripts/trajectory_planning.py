import sim
import numpy as np

def connect_to_simulator():
    sim.simxFinish(-1)  # Fecha todas as conexões existentes
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conecta ao CoppeliaSim
    if clientID != -1:
        print('Conexão estabelecida com sucesso!')
    else:
        print('Falha ao conectar ao CoppeliaSim.')
    return clientID

def disconnect_from_simulator(clientID):
    sim.simxFinish(clientID)
    print('Conexão encerrada.')

def get_object_handle(clientID, object_name):
    _, handle = sim.simxGetObjectHandle(clientID, object_name, sim.simx_opmode_blocking)
    return handle

def set_joint_positions(clientID, joint_handles, joint_positions):
    for i, handle in enumerate(joint_handles):
        sim.simxSetJointTargetPosition(clientID, handle, joint_positions[i], sim.simx_opmode_oneshot)

def plan_cubic_trajectory(start_position, end_position, num_points):
    t = np.linspace(0, 1, num_points)
    trajectory = (1 - t[:, None]) * start_position + t[:, None] * end_position
    return trajectory

def main():
    clientID = connect_to_simulator()

    # Obter handles das juntas do robô UR5
    joint_handles = []
    for i in range(1, 7):
        joint_name = 'UR5_joint' + str(i)
        joint_handle = get_object_handle(clientID, joint_name)
        joint_handles.append(joint_handle)

    # Definir uma trajetória cúbica desejada
    start_position = [0, 0, 0, 0, 0, 0]
    end_position = [np.pi/2, np.pi/4, np.pi/2, np.pi/4, np.pi/2, 0]
    num_points = 200
    trajectory = plan_cubic_trajectory(start_position, end_position, num_points)

    # Executar a trajetória
    for point in trajectory:
        set_joint_positions(clientID, joint_handles, point)
        sim.simxSynchronousTrigger(clientID)  # Dispara a próxima etapa da simulação
        sim.simxGetPingTime(clientID)  # Espera até a simulação ser concluída

    disconnect_from_simulator(clientID)

if __name__ == '__main__':
    main()

