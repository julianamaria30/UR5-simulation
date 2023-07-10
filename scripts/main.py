from UR5Kinematics import UR5Kinematics
from UR5Simulation import UR5Simulation
import numpy as np
import time

def main():
    # Etapa A1:
    # - Implementar a Cinemática Direta e Inversa do Robô UR5. Validar a modelagem com o auxílio do
    #   simulador para um conjunto de valores das variáveis das juntas (Cinemática Direta) e para um conjunto de
    #   posições e orientações da garra (Cinemática Inversa)

    # Exemplo para fins de demonstrações das equações da cinemática direta e cinemática inversa
    
    ur5 = UR5Kinematics()
    simulation = UR5Simulation()

    targetPos1 = [90 * np.pi / 180, 90 * np.pi / 180, -90 * np.pi / 180, 90 * np.pi / 180, 90 * np.pi / 180,
                  90 * np.pi / 180]
    Cinematica_Direta = ur5.forward_kinematics(targetPos1)
    print(Cinematica_Direta)
    Cinematica_Inversa = ur5.inverse_kinematics(Cinematica_Direta, elbow = 'down')
    print(Cinematica_Inversa)
    simulation.set_joint_angles(Cinematica_Inversa)

    time.sleep(3)

    targetPos2=[-90*np.pi/180,45*np.pi/180,90*np.pi/180,135*np.pi/180,90*np.pi/180,90*np.pi/180]
    Cinematica_Direta = ur5.forward_kinematics(targetPos2)
    print(Cinematica_Direta)
    Cinematica_Inversa = ur5.inverse_kinematics(Cinematica_Direta)
    print(Cinematica_Inversa)
    simulation.set_joint_angles(Cinematica_Inversa)


    time.sleep(3)

    targetPos3 = [0,0,0,0,0,0]
    Cinematica_Direta = ur5.forward_kinematics(targetPos3)
    print(Cinematica_Direta)
    Cinematica_Inversa = ur5.inverse_kinematics(Cinematica_Direta)
    print(Cinematica_Inversa)
    simulation.set_joint_angles(Cinematica_Inversa)

if __name__ == '__main__':
    main()