import re


def parse_command(command):

    command = command.lower().strip()

    # ------------------------------------------------
    # Cartesian position command
    # Example: move tcp to x 0.1 m y 0.3 m z 0.15 m
    # ------------------------------------------------

    if "tcp" in command:

        pattern = (
            r"x\s*(-?\d+\.?\d*)\s*m.*?"
            r"y\s*(-?\d+\.?\d*)\s*m.*?"
            r"z\s*(-?\d+\.?\d*)\s*m"
        )

        match = re.search(pattern, command)

        if not match:
            return None, "Invalid Cartesian command."

        x, y, z = map(float, match.groups())

        return {
            "type": "cartesian",
            "target": (x, y, z)
        }, "ok"

    # ------------------------------------------------
    # Joint-space command
    # Example: move joint1 to 10 degrees
    #          and joint2 to 0.3 m
    # ------------------------------------------------

    joint1_pattern = (
        r"(?:joint\s*1|first\s+joint|theta1)"
        r"\s*(?:to|=)?\s*"
        r"(-?\d+\.?\d*)\s*"
        r"(degrees?|deg|radians?|rad)"
    )

    joint2_pattern = (
        r"(?:joint\s*2|second\s+joint|d2)"
        r"\s*(?:to|=)?\s*"
        r"(-?\d+\.?\d*)\s*"
        r"(m|meters?|cm|mm)"
    )

    match1 = re.search(joint1_pattern, command)
    match2 = re.search(joint2_pattern, command)

    if match1 and match2:

        angle = float(match1.group(1))
        angle_unit = match1.group(2)

        extension = float(match2.group(1))
        extension_unit = match2.group(2)

        # Convert angle to radians
        if angle_unit in ("degree", "degrees", "deg"):
            import numpy as np
            angle = np.deg2rad(angle)

        # Convert extension to metres
        if extension_unit == "cm":
            extension /= 100

        elif extension_unit == "mm":
            extension /= 1000

        return {
            "type": "joint",
            "theta1": angle,
            "d2": extension
        }, "ok"

    return None, "Command not recognized."