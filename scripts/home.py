import sim

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

def move_to_home_position(clientID, joint_handles):
    home_positions = [0, -0.5, 0.5, -1.3, -1.6, 0]  # Posição "home" desejada para cada junta
    set_joint_positions(clientID, joint_handles, home_positions)
    sim.simxSynchronousTrigger(clientID)  # Dispara a próxima etapa da simulação
    sim.simxGetPingTime(clientID)  # Espera até a simulação ser concluída

def main():
    clientID = connect_to_simulator()

    # Obter handles das juntas do robô UR5
    joint_handles = []
    for i in range(1, 7):
        joint_name = 'UR5_joint' + str(i)
        joint_handle = get_object_handle(clientID, joint_name)
        joint_handles.append(joint_handle)

    move_to_home_position(clientID, joint_handles)

    disconnect_from_simulator(clientID)

if __name__ == '__main__':
    main()
