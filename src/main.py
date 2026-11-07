import queue
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from command_parser import parse_command

PROJECT_DIR = Path(__file__).resolve().parent.parent
MODEL_PATH = PROJECT_DIR / "assets" / "robot_arm.xml"

TARGET = (0.1, 0.3, 0.15)


def get_tcp_reference(model):
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)

    site_id = model.site("tool_site").id
    position = data.site_xpos[site_id].copy()

    radius = np.hypot(position[0], position[1])
    return float(radius), float(position[2])


def inverse_kinematics(target, min_radius, tcp_height, max_extension):
    x, y, z = target

    if abs(z - tcp_height) > 0.005:
        return None

    radius = np.hypot(x, y)
    extension = radius - min_radius

    if not 0 <= extension <= max_extension:
        return None

    angle = np.arctan2(y, x) % (2 * np.pi)
    return float(angle), float(extension)


def read_commands(commands):
    while True:
        try:
            text = input("Robot > ").strip()

            if text.lower() == "exit":
                break

            if not text:
                continue

            command, message = parse_command(text)

            if command is None:
                print(message)
                continue

            commands.put(command)

        except EOFError:
            break


def execute_command(command, data, motors, limits, workspace):
    motor1, motor2 = motors
    lo1, hi1, lo2, hi2 = limits
    min_radius, tcp_height = workspace

    if command["type"] == "joint":
        angle = command["theta1"]
        extension = command["d2"]

    elif command["type"] == "cartesian":
        solution = inverse_kinematics(
            command["target"],
            min_radius,
            tcp_height,
            hi2
        )

        if solution is None:
            print("Target is outside the robot workspace.")
            return

        angle, extension = solution

    else:
        print("Unknown command.")
        return

    if not lo1 <= angle <= hi1:
        print("Joint 1 target is outside its limits.")
        return

    if not lo2 <= extension <= hi2:
        print("Joint 2 target is outside its limits.")
        return

    data.ctrl[motor1] = angle
    data.ctrl[motor2] = extension

    print(
        f"Target accepted: "
        f"Joint 1 = {np.rad2deg(angle):.2f} degrees, "
        f"Joint 2 = {extension:.3f} m"
    )


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    joint1 = model.joint("theta1").id
    joint2 = model.joint("d2").id

    qpos1 = model.jnt_qposadr[joint1]
    qpos2 = model.jnt_qposadr[joint2]

    motor1 = model.actuator("theta1_act").id
    motor2 = model.actuator("d2_act").id

    lo1, hi1 = model.actuator_ctrlrange[motor1]
    lo2, hi2 = model.actuator_ctrlrange[motor2]

    min_radius, tcp_height = get_tcp_reference(model)

    angle = np.deg2rad(45)
    extension = 0.40

    if TARGET is not None:
        solution = inverse_kinematics(
            TARGET, min_radius, tcp_height, hi2
        )

        if solution is not None:
            angle, extension = solution
        else:
            print("Initial target is outside the workspace.")

    data.qpos[qpos1] = angle
    data.qpos[qpos2] = extension

    data.ctrl[motor1] = angle
    data.ctrl[motor2] = extension

    mujoco.mj_forward(model, data)

    tool_site = model.site("tool_site").id
    actual_position = data.site_xpos[tool_site].copy()

    expected_position = np.array([
        (min_radius + extension) * np.cos(angle),
        (min_radius + extension) * np.sin(angle),
        tcp_height
    ])

    error = np.linalg.norm(actual_position - expected_position)

    print("\nRobot loaded successfully.")
    print(f"Joints: {model.njnt}")
    print(f"Actuators: {model.nu}")
    print(f"TCP position: {actual_position}")
    print(f"FK error: {error:.8f} m")

    commands = queue.Queue()

    listener = threading.Thread(
        target=read_commands,
        args=(commands,),
        daemon=True
    )
    listener.start()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.25, 0.0, 0.15]
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20

        while viewer.is_running():
            start = time.time()

            while not commands.empty():
                command = commands.get()

                execute_command(
                    command,
                    data,
                    (motor1, motor2),
                    (lo1, hi1, lo2, hi2),
                    (min_radius, tcp_height)
                )

            mujoco.mj_step(model, data)
            viewer.sync()

            remaining = model.opt.timestep - (time.time() - start)

            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()