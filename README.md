# trash

def limit_velocities(self, vel_x, vel_y, vel_z, max_value, ignore_z_axis=False, logger=False):
        vx = 0.0
        vy = 0.0
        vz = 0.0
        if ignore_z_axis:
            magnitude = np.linalg.norm([vel_x, vel_y, vel_z])

            if magnitude > 0:
                unit_vector = np.array([vel_x, vel_y, vel_z]) / magnitude
            else:
                unit_vector = np.array([0.0, 0.0, 0.0])

            # clamp magnitude

            if magnitude > max_value:
                # print("[INFO] Setting maximum speed to ", max_value)
                magnitude = max_value
                vx, vy, vz = unit_vector*magnitude

            if logger:
                timestamp = datetime.datetime.now().isoformat(timespec="seconds")
                print(
                    f"[{timestamp}][LOG][normalize] "
                    f"magnitude: {magnitude}, "
                    f"unit_vector: {unit_vector}, "
                    f"scaled_velocity x: {vx}, y: {vy}, z: {vz}"
                )

            return vx, vy, 0.0
        else:
            return vx, vy, vz
