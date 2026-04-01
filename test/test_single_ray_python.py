"""
skyratio_calc を Python から直接呼んで、
worker.py と同じ box/checkpoint 条件で 1 点だけ確認する最小スクリプト。
"""

import skyratio_calc
print(skyratio_calc.__file__)


def main():
    origin = [0.0, -2.0, 0.0]

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

    checker = skyratio_calc.SkyRatioChecker()
    checker.ray_resolution = 10.0
    checker.checkpoints = [origin]

    sky_ratios = checker.check(scene)

    print(f"origin={origin}")
    print(f"box_position={box_position}")
    print(f"box_size={box_size}")
    print(f"box_center={box_center}")
    print(f"box_euler={box_euler}")
    print(f"ray_resolution={checker.ray_resolution}")
    print(f"sky_ratios={sky_ratios}")


if __name__ == "__main__":
    main()
