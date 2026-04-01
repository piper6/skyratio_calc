"""
skyratio_calc を Python から直接呼んで、
ray=0011 だけを scene.raycast() で判定する最小スクリプト。
"""

import skyratio_calc


def main():
    origin = [0.0, -2.0, 0.0]
    direction = [0.044290756, 0.141205909, 0.988988989]  # ray=0011

    # compare_new_old2.py の現在設定を worker.py と同じ方法で serialize
    box_position = [0.01, 0.0, 0.0]
    box_size = [16.5, 24.0, 32.0]
    box_yaw = 0.0

    box_center = [
        box_position[0] + box_size[0] / 2.0,
        box_position[1] + box_size[1] / 2.0,
        box_position[2] + box_size[2] / 2.0,
    ]
    box_euler = [0.0, 0.0, box_yaw]

    scene = skyratio_calc.SceneRaycaster()
    scene.add_box(box_center, box_size, box_euler)
    scene.build()

    results = scene.raycast([origin], [direction])
    hit = results[0]

    print(f"origin={origin}")
    print(f"direction={direction}")
    print(f"box_position={box_position}")
    print(f"box_size={box_size}")
    print(f"box_center={box_center}")
    print(f"box_euler={box_euler}")
    print(f"hit={hit.hit}")
    print(f"distance={hit.distance}")
    print(f"position={hit.position}")


if __name__ == "__main__":
    main()
