import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "assets" / "robot_arm.xml"

DEMO_MOTION = False
RUN_SELF_TEST = True
TARGET = (0.1, 0.3, 0.15)
Z_TOLERANCE = 0.005


def get_tcp_reference(model):
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    site_id = model.site("tool_site").id
    tcp = data.site_xpos[site_id].copy()
    return float(np.hypot(tcp[0], tcp[1])), float(tcp[2])


def inverse_kinematics(target, r_min, z_tcp, d2_max):
    x, y, z = target
    r = float(np.hypot(x, y))

    if abs(z - z_tcp) > Z_TOLERANCE:
        return None, f"unreachable: height z={z:.3f} m, arm works at z={z_tcp:.3f} m"

    d2 = r - r_min
    if d2 < 0.0 or d2 > d2_max:
        return None, (f"unreachable: radius {r:.3f} m, "
                      f"allowed {r_min:.3f} to {r_min + d2_max:.3f} m")

    theta1 = float(np.arctan2(y, x)) % (2.0 * np.pi)
    return (theta1, d2), "ok"


def run_self_test(model):
    data = mujoco.MjData(model)
    site_id = model.site("tool_site").id

    print("\nKeyframe check")
    for key_id in range(model.nkey):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_id)
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        mujoco.mj_forward(model, data)
        tcp = data.site_xpos[site_id]
        print(f"{name:10s} -> x={tcp[0]: .4f}  y={tcp[1]: .4f}  z={tcp[2]: .4f}")


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint1_id = model.joint("theta1").id
    joint2_id = model.joint("d2").id

    qpos1 = model.jnt_qposadr[joint1_id]
    qpos2 = model.jnt_qposadr[joint2_id]

    motor1 = model.actuator("theta1_act").id
    motor2 = model.actuator("d2_act").id

    lo1, hi1 = model.actuator_ctrlrange[motor1]
    lo2, hi2 = model.actuator_ctrlrange[motor2]

    tool_site = model.site("tool_site").id

    r_min, z_tcp = get_tcp_reference(model)

    print("\nRobot loaded successfully!")
    print("Joints:", model.njnt)
    print("Actuators:", model.nu)
    print(f"TCP height : {z_tcp:.4f} m")
    print(f"TCP reach  : {r_min:.4f} to {r_min + hi2:.4f} m")

    if RUN_SELF_TEST:
        run_self_test(model)

    angle = np.deg2rad(45)
    extension = 0.40

    if TARGET is not None:
        solution, message = inverse_kinematics(TARGET, r_min, z_tcp, hi2)
        print(f"\nTarget {TARGET}: {message}")
        if solution is not None:
            angle, extension = solution

    angle = float(np.clip(angle, lo1, hi1))
    extension = float(np.clip(extension, lo2, hi2))

    data.qpos[qpos1] = angle
    data.qpos[qpos2] = extension

    data.ctrl[motor1] = angle
    data.ctrl[motor2] = extension

    mujoco.mj_forward(model, data)

    print("\nInitial Configuration")
    print("theta1 :", np.rad2deg(angle), "degrees")
    print("d2     :", extension, "meters")
    print("Tool   :", data.site_xpos[tool_site].copy())

    with mujoco.viewer.launch_passive(model, data) as viewer:

        viewer.cam.lookat[:] = [0.25, 0.0, 0.15]
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        t0 = time.time()

        while viewer.is_running():
            step_start = time.time()

            if DEMO_MOTION:
                t = time.time() - t0
                data.ctrl[motor1] = np.clip(
                    np.pi * (1 + np.sin(0.4 * t - np.pi / 2)), lo1, hi1)
                data.ctrl[motor2] = np.clip(
                    0.2 * (1 + np.sin(0.8 * t - np.pi / 2)), lo2, hi2)

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()