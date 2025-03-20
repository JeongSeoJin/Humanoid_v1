import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink
import matplotlib.pyplot as plt

def main():
    # === (1) 4 DOF 체인 정의 (ikpy 3.4.2 호환) ===
    four_dof_chain = Chain(name="4dof_arm", links=[
        URDFLink(
            name="base_link",
            origin_translation=[0.0, 0.0, 0.0],
            origin_orientation=[0.0, 0.0, 0.0],
            rotation=[0, 0, 1],  # Z축
        ),
        URDFLink(
            name="joint_1",
            origin_translation=[0.2, 0.0, 0.0],
            origin_orientation=[0.0, 0.0, 0.0],
            rotation=[0, 0, 1],
        ),
        URDFLink(
            name="joint_2",
            origin_translation=[0.2, 0.0, 0.0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
        ),
        URDFLink(
            name="joint_3",
            origin_translation=[0.1, 0.0, 0.0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
        ),
        URDFLink(
            name="joint_4_end_effector",
            origin_translation=[0.1, 0.0, 0.0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
        ),
    ])

    # === (2) 목표 좌표 & IK ===
    target_position = [0.5, 0.0, 0.0]
    initial_joints = [0, 0, 0, 0, 0]  # 체인 links=5개
    ik_solution = four_dof_chain.inverse_kinematics(
        target_position=target_position,
        initial_position=initial_joints
    )
    
    print("=== IK result (rad) ===")
    for i, angle in enumerate(ik_solution):
        print(f" Link {i}: {angle:.3f} rad ({np.rad2deg(angle):.1f} deg)")

    # (3) 최종 FK -> End Effector 좌표
    final_transform = four_dof_chain.forward_kinematics(ik_solution)
    ee_pos = final_transform[0:3, 3]
    print("\nEnd Effector final pos:", ee_pos)

    # (4) **각 링크별 변환행렬** : forward_kinematics(..., index=i)
    chain_positions = []
    for link_index in range(len(four_dof_chain.links)):
        # 0~4까지 순회
        partial_transform = four_dof_chain.forward_kinematics(ik_solution, index=link_index)
        x = partial_transform[0, 3]
        y = partial_transform[1, 3]
        chain_positions.append((x, y))

    # (5) 2D 플롯
    xs = [p[0] for p in chain_positions]
    ys = [p[1] for p in chain_positions]
    plt.figure()
    plt.plot(xs, ys, marker='o', linestyle='-')
    # 목표점 표시
    plt.scatter(target_position[0], target_position[1], c='r', label='Target')
    plt.xlabel("X (m)"); plt.ylabel("Y (m)")
    plt.title("4-DOF IK with ikpy 3.4.2: each link transform via forward_kinematics(index=i)")
    plt.grid(True); plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
