# Esse código foi desenvolvido para facilitar os testes com o robô. Aqui é possíve enviar as seis posições das juntas via teclado.
# Os valores de posição das juntas devem ser enviados em graus.

import vrep
import numpy as np

def connect_to_simulator():
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Conecta ao V-REP
    if clientID == -1:
        print('Falha ao conectar ao V-REP.')
        return -1
    print('Conexão estabelecida com sucesso!')
    return clientID

def disconnect_from_simulator(clientID):
    vrep.simxFinish(clientID)
    print('Conexão encerrada.')

def get_joint_handles(clientID):
    joint_handles = []
    for i in range(1, 7):
        joint_name = 'UR5_joint' + str(i)
        _, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_blocking)
        joint_handles.append(handle)
    return joint_handles

def set_joint_positions(clientID, joint_handles, joint_positions):
    for handle, position in zip(joint_handles, joint_positions):
        vrep.simxSetJointTargetPosition(clientID, handle, position, vrep.simx_opmode_oneshot)

def get_user_input():
    joint_positions = []
    for i in range(6):
        position = float(input(f'Insira a posição da junta {i+1} em graus: '))
        position_radians = np.radians(position)
        joint_positions.append(position_radians)
    return joint_positions

def main():
    clientID = connect_to_simulator()
    if clientID == -1:
        return

    joint_handles = get_joint_handles(clientID)

    joint_positions = get_user_input()

    set_joint_positions(clientID, joint_handles, joint_positions)

    # Obter as posições atualizadas das juntas
    updated_joint_positions = []
    for handle in joint_handles:
        _, position = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
        updated_joint_positions.append(position)

    # Exibir as posições das juntas
    print('Posições das juntas:')
    for i, position in enumerate(updated_joint_positions):
        position_degrees = np.degrees(position)
        print(f'Junta {i+1}: {position_degrees} graus')

    disconnect_from_simulator(clientID)

if __name__ == '__main__':
    main()
