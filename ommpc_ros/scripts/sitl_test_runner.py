#!/usr/bin/env python3

"""Run one PX4 SITL tracking trial and write synchronized RMSE results."""

import argparse
import csv
import json
import threading
import time
from pathlib import Path

import numpy as np
import rospy
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from mavros_msgs.msg import AttitudeTarget, ParamValue, State
from mavros_msgs.srv import ParamSet, SetMode
from nav_msgs.msg import Odometry
from traj_utils.msg import PolyTraj


def rotate_vector(q, vector):
    q_vector = np.array([q.x, q.y, q.z], dtype=float)
    vector = np.asarray(vector, dtype=float)
    return vector + 2.0 * q.w * np.cross(q_vector, vector) + 2.0 * np.cross(
        q_vector, np.cross(q_vector, vector)
    )


def clamped_cubic_coefficients(waypoints, durations):
    """Return per-piece coefficients in the descending order expected by PolyTraj."""
    waypoints = np.asarray(waypoints, dtype=float)
    durations = np.asarray(durations, dtype=float)
    segment_count = durations.size
    matrix = np.zeros((segment_count + 1, segment_count + 1))
    rhs = np.zeros((segment_count + 1, 3))
    slopes = np.diff(waypoints, axis=0) / durations[:, None]

    matrix[0, 0] = 2.0 * durations[0]
    matrix[0, 1] = durations[0]
    rhs[0] = 6.0 * slopes[0]
    matrix[-1, -2] = durations[-1]
    matrix[-1, -1] = 2.0 * durations[-1]
    rhs[-1] = -6.0 * slopes[-1]
    for i in range(1, segment_count):
        matrix[i, i - 1] = durations[i - 1]
        matrix[i, i] = 2.0 * (durations[i - 1] + durations[i])
        matrix[i, i + 1] = durations[i]
        rhs[i] = 6.0 * (slopes[i] - slopes[i - 1])

    second_derivatives = np.linalg.solve(matrix, rhs)
    coefficients = []
    for i, duration in enumerate(durations):
        c0 = waypoints[i]
        c1 = slopes[i] - duration * (
            2.0 * second_derivatives[i] + second_derivatives[i + 1]
        ) / 6.0
        c2 = second_derivatives[i] / 2.0
        c3 = (second_derivatives[i + 1] - second_derivatives[i]) / (6.0 * duration)
        coefficients.append(np.stack((c3, c2, c1, c0), axis=1))
    return coefficients


def c3_septic_coefficients(waypoints, durations, knot_speed_limit):
    """Upgrade a clamped cubic scaffold to C3-continuous seventh-order pieces."""
    cubic = clamped_cubic_coefficients(waypoints, durations)
    segment_count = len(cubic)
    velocities = np.zeros_like(waypoints)
    accelerations = np.zeros_like(waypoints)
    jerks = np.zeros_like(waypoints)

    velocities[0] = cubic[0][:, 2]
    for i, (coefficient, duration) in enumerate(zip(cubic, durations)):
        velocities[i + 1] = (
            3.0 * coefficient[:, 0] * duration**2
            + 2.0 * coefficient[:, 1] * duration
            + coefficient[:, 2]
        )
    for i in range(1, segment_count):
        speed = np.linalg.norm(velocities[i])
        if speed > knot_speed_limit:
            velocities[i] *= knot_speed_limit / speed

    coefficients = []
    for i, duration in enumerate(durations):
        ascending = np.zeros((8, 3))
        ascending[0] = waypoints[i]
        ascending[1] = velocities[i]
        ascending[2] = 0.5 * accelerations[i]
        ascending[3] = jerks[i] / 6.0

        matrix = np.array(
            [
                [duration**4, duration**5, duration**6, duration**7],
                [4.0 * duration**3, 5.0 * duration**4, 6.0 * duration**5, 7.0 * duration**6],
                [12.0 * duration**2, 20.0 * duration**3, 30.0 * duration**4, 42.0 * duration**5],
                [24.0 * duration, 60.0 * duration**2, 120.0 * duration**3, 210.0 * duration**4],
            ]
        )
        endpoint_time = np.array([1.0, duration, duration**2, duration**3])
        endpoint_position = endpoint_time @ ascending[:4]
        endpoint_velocity = ascending[1] + 2.0 * ascending[2] * duration + 3.0 * ascending[3] * duration**2
        endpoint_acceleration = 2.0 * ascending[2] + 6.0 * ascending[3] * duration
        endpoint_jerk = 6.0 * ascending[3]
        rhs = np.stack(
            (
                waypoints[i + 1] - endpoint_position,
                velocities[i + 1] - endpoint_velocity,
                accelerations[i + 1] - endpoint_acceleration,
                jerks[i + 1] - endpoint_jerk,
            )
        )
        ascending[4:] = np.linalg.solve(matrix, rhs)
        coefficients.append(ascending[::-1].T)
    return coefficients


def evaluate_polynomial_derivative(coefficient, time_samples, derivative_order):
    degree = coefficient.shape[1] - 1
    result = np.zeros((coefficient.shape[0], time_samples.size))
    for column, power in enumerate(range(degree, -1, -1)):
        if power < derivative_order:
            continue
        multiplier = 1.0
        for offset in range(derivative_order):
            multiplier *= power - offset
        result += (
            multiplier
            * coefficient[:, column, None]
            * time_samples[None, :] ** (power - derivative_order)
        )
    return result


def polynomial_peak_speed(coefficients, durations):
    peak_speed = 0.0
    for coefficient, duration in zip(coefficients, durations):
        time_samples = np.linspace(0.0, duration, 1001)
        velocity = evaluate_polynomial_derivative(coefficient, time_samples, 1)
        peak_speed = max(peak_speed, float(np.linalg.norm(velocity, axis=0).max()))
    return peak_speed


def polynomial_reference_limits(
    coefficients,
    durations,
    drag_mass,
    horizontal_drag,
    vertical_drag,
    drag_speed_coefficient,
):
    maximum_acceleration = 0.0
    maximum_thrust = 0.0
    gravity = np.array([0.0, 0.0, 9.81])
    for coefficient, duration in zip(coefficients, durations):
        time_samples = np.linspace(0.0, duration, 1001)
        velocity = evaluate_polynomial_derivative(coefficient, time_samples, 1).T
        acceleration = evaluate_polynomial_derivative(coefficient, time_samples, 2).T
        drag_scale = 1.0 + drag_speed_coefficient * np.sqrt(
            np.sum(velocity**2, axis=1) + 0.02
        )
        scaled_velocity = drag_scale[:, None] * velocity
        body_z_unnormalized = (
            acceleration + gravity + horizontal_drag / drag_mass * scaled_velocity
        )
        body_z = body_z_unnormalized / np.linalg.norm(body_z_unnormalized, axis=1)[:, None]
        thrust = np.sum(
            body_z
            * (acceleration + gravity + vertical_drag / drag_mass * scaled_velocity),
            axis=1,
        )
        maximum_acceleration = max(
            maximum_acceleration, float(np.linalg.norm(acceleration, axis=1).max())
        )
        maximum_thrust = max(maximum_thrust, float(thrust.max()))
    return maximum_acceleration, maximum_thrust


def make_polynomial_message(
    start_time,
    target_peak_speed,
    base_altitude,
    drag_mass,
    horizontal_drag,
    vertical_drag,
    drag_speed_coefficient,
    thrust_to_weight_limit=1.30,
):
    # A C3-continuous, three-dimensional route with zero velocity at both ends.
    waypoints = np.array(
        [
            [0.0, 0.0, 0.8],
            [2.2, -0.8, 1.2],
            [4.0, 1.5, 1.8],
            [2.0, 4.0, 1.1],
            [-1.0, 2.5, 1.6],
            [0.0, 0.0, 0.8],
        ]
    )
    waypoints[:, 2] += base_altitude - 0.8
    base_durations = np.array([1.2, 1.3, 1.4, 1.45, 1.25])
    base_durations[1:] *= max(1.0, target_peak_speed / 10.0)

    def coefficients_for_scale(horizontal_scale, durations):
        scaled_waypoints = waypoints.copy()
        scaled_waypoints[:, :2] *= horizontal_scale
        if horizontal_scale > 0.0:
            # Align the smooth initial tangent with the takeoff yaw. This avoids
            # introducing a yaw step while the vehicle is only starting to move.
            provisional = c3_septic_coefficients(
                scaled_waypoints, durations, 0.25 * target_peak_speed
            )
            initial_motion = provisional[0][:2, 3]
            initial_heading = np.arctan2(initial_motion[1], initial_motion[0])
            cosine = np.cos(-initial_heading)
            sine = np.sin(-initial_heading)
            heading_rotation = np.array([[cosine, -sine], [sine, cosine]])
            scaled_waypoints[:, :2] = scaled_waypoints[:, :2] @ heading_rotation.T
        return c3_septic_coefficients(
            scaled_waypoints, durations, 0.25 * target_peak_speed
        )

    def fit_peak_speed(time_scale):
        durations = base_durations * time_scale
        if polynomial_peak_speed(coefficients_for_scale(0.0, durations), durations) > target_peak_speed:
            return None
        lower_scale = 0.0
        upper_scale = 1.0
        while (
            polynomial_peak_speed(coefficients_for_scale(upper_scale, durations), durations)
            < target_peak_speed
        ):
            upper_scale *= 2.0
        for _ in range(32):
            middle_scale = 0.5 * (lower_scale + upper_scale)
            if (
                polynomial_peak_speed(coefficients_for_scale(middle_scale, durations), durations)
                < target_peak_speed
            ):
                lower_scale = middle_scale
            else:
                upper_scale = middle_scale
        return coefficients_for_scale(upper_scale, durations), durations, upper_scale

    # Leave more attitude/thrust margin as speed and aerodynamic load increase.
    # The four test points use 4 m/s^2 at 8/12 m/s, 3 at 16 m/s, and 2 at 20 m/s.
    if target_peak_speed <= 12.0:
        maximum_reference_acceleration = 4.0
    elif target_peak_speed <= 16.0:
        maximum_reference_acceleration = 3.0
    else:
        maximum_reference_acceleration = 2.0
    maximum_reference_thrust_to_weight = thrust_to_weight_limit
    lower_time_scale = 1.0
    upper_time_scale = 1.0
    while True:
        fitted = fit_peak_speed(upper_time_scale)
        if fitted is not None:
            coefficients, durations, horizontal_scale = fitted
            maximum_acceleration, maximum_thrust = polynomial_reference_limits(
                coefficients,
                durations,
                drag_mass,
                horizontal_drag,
                vertical_drag,
                drag_speed_coefficient,
            )
            if (
                maximum_acceleration <= maximum_reference_acceleration
                and maximum_thrust / 9.81 <= maximum_reference_thrust_to_weight
            ):
                break
        lower_time_scale = upper_time_scale
        upper_time_scale *= 1.5
        if upper_time_scale > 128.0:
            raise RuntimeError("requested speed is infeasible under the reference limits")

    for _ in range(28):
        middle_time_scale = 0.5 * (lower_time_scale + upper_time_scale)
        fitted = fit_peak_speed(middle_time_scale)
        if fitted is None:
            lower_time_scale = middle_time_scale
            continue
        candidate_coefficients, candidate_durations, candidate_horizontal_scale = fitted
        candidate_acceleration, candidate_thrust = polynomial_reference_limits(
            candidate_coefficients,
            candidate_durations,
            drag_mass,
            horizontal_drag,
            vertical_drag,
            drag_speed_coefficient,
        )
        if (
            candidate_acceleration > maximum_reference_acceleration
            or candidate_thrust / 9.81 > maximum_reference_thrust_to_weight
        ):
            lower_time_scale = middle_time_scale
        else:
            upper_time_scale = middle_time_scale
            coefficients = candidate_coefficients
            durations = candidate_durations
            horizontal_scale = candidate_horizontal_scale
            maximum_acceleration = candidate_acceleration
            maximum_thrust = candidate_thrust

    msg = PolyTraj()
    msg.drone_id = 0
    msg.traj_id = 1
    msg.start_time = start_time
    msg.order = coefficients[0].shape[1] - 1
    msg.duration = durations.astype(np.float32).tolist()
    for coefficient in coefficients:
        msg.coef_x.extend(coefficient[0].astype(np.float32).tolist())
        msg.coef_y.extend(coefficient[1].astype(np.float32).tolist())
        msg.coef_z.extend(coefficient[2].astype(np.float32).tolist())
    metadata = {
        "planned_horizontal_scale": float(horizontal_scale),
        "planned_time_scale": float(upper_time_scale),
        "planned_duration_s": float(durations.sum()),
        "planned_acceleration_limit_mps2": float(maximum_reference_acceleration),
        "planned_feedforward_thrust_to_weight_limit": float(
            maximum_reference_thrust_to_weight
        ),
        "planned_max_acceleration_mps2": float(maximum_acceleration),
        "planned_max_feedforward_thrust_to_weight": float(maximum_thrust / 9.81),
    }
    return msg, float(durations.sum()), metadata


class TrialRunner:
    def __init__(self, velocity_in_body_frame):
        self._lock = threading.Lock()
        self._state = None
        self._odom = None
        self._record_start = None
        self._record_end = None
        self._samples = []
        self._control_command = None
        self._velocity_in_body_frame = velocity_in_body_frame

        rospy.Subscriber("/mavros/state", State, self._state_callback, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_callback, queue_size=50)
        rospy.Subscriber("/ommpc_controller/reference", Odometry, self._reference_callback, queue_size=50)
        rospy.Subscriber(
            "/mavros/setpoint_raw/attitude",
            AttitudeTarget,
            self._control_command_callback,
            queue_size=50,
        )

    def _state_callback(self, msg):
        with self._lock:
            self._state = msg

    def _odom_callback(self, msg):
        with self._lock:
            self._odom = msg

    def _control_command_callback(self, msg):
        with self._lock:
            self._control_command = np.array(
                [msg.thrust, msg.body_rate.x, msg.body_rate.y, msg.body_rate.z]
            )

    def _reference_callback(self, msg):
        stamp = msg.header.stamp.to_sec()
        with self._lock:
            if (
                self._record_start is None
                or stamp < self._record_start
                or stamp > self._record_end
                or self._odom is None
            ):
                return
            odom = self._odom
            control_command = self._control_command

        reference_position = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        )
        actual_position = np.array(
            [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
        )
        reference_velocity = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        )
        actual_velocity = np.array(
            [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]
        )
        if self._velocity_in_body_frame:
            actual_velocity = rotate_vector(odom.pose.pose.orientation, actual_velocity)
        reference_quaternion = np.array(
            [
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        actual_quaternion = np.array(
            [
                odom.pose.pose.orientation.w,
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
            ]
        )

        sample = np.concatenate(
            (
                [stamp],
                reference_position,
                actual_position,
                reference_velocity,
                actual_velocity,
                np.full(4, np.nan) if control_command is None else control_command,
                reference_quaternion,
                actual_quaternion,
            )
        )
        with self._lock:
            self._samples.append(sample)

    def state(self):
        with self._lock:
            return self._state

    def altitude(self):
        with self._lock:
            return None if self._odom is None else self._odom.pose.pose.position.z

    def begin_recording(self, start, duration):
        with self._lock:
            self._samples = []
            self._record_start = start
            self._record_end = start + duration

    def samples(self):
        with self._lock:
            return np.asarray(self._samples, dtype=float)


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        rospy.rostime.wallsleep(0.05)
    raise RuntimeError("timed out waiting for {}".format(description))


def compute_metrics(
    samples,
    warmup,
    drag_mass,
    horizontal_drag,
    vertical_drag,
    drag_speed_coefficient,
):
    if samples.ndim != 2 or samples.shape[0] < 20:
        raise RuntimeError("too few synchronized samples: {}".format(samples.shape[0]))
    reference_velocity_all = samples[:, 7:10]
    reference_time_all = samples[:, 0] - samples[0, 0]
    reference_acceleration_all = np.gradient(
        reference_velocity_all, samples[:, 0], axis=0
    )
    gravity = np.array([0.0, 0.0, 9.81])
    drag_scale = 1.0 + drag_speed_coefficient * np.sqrt(
        np.sum(reference_velocity_all**2, axis=1) + 0.02
    )
    scaled_reference_velocity = drag_scale[:, None] * reference_velocity_all
    body_z_unnormalized = (
        reference_acceleration_all
        + gravity
        + horizontal_drag / drag_mass * scaled_reference_velocity
    )
    body_z = body_z_unnormalized / np.linalg.norm(body_z_unnormalized, axis=1)[:, None]
    feedforward_thrust = np.sum(
        body_z
        * (
            reference_acceleration_all
            + gravity
            + vertical_drag / drag_mass * scaled_reference_velocity
        ),
        axis=1,
    )
    reference_acceleration_norm = np.linalg.norm(reference_acceleration_all, axis=1)
    max_acceleration_index = int(np.argmax(reference_acceleration_norm))

    samples = samples[samples[:, 0] >= samples[0, 0] + warmup]
    position_error = samples[:, 4:7] - samples[:, 1:4]
    velocity_error = samples[:, 10:13] - samples[:, 7:10]
    position_norm = np.linalg.norm(position_error, axis=1)
    velocity_norm = np.linalg.norm(velocity_error, axis=1)
    reference_speed = np.linalg.norm(samples[:, 7:10], axis=1)
    control_command = samples[:, 13:17]
    valid_control = np.all(np.isfinite(control_command), axis=1)

    metrics = {
        "samples": int(samples.shape[0]),
        "measured_duration_s": float(samples[-1, 0] - samples[0, 0]),
        "position_rmse_m": float(np.sqrt(np.mean(position_norm**2))),
        "position_axis_rmse_m": np.sqrt(np.mean(position_error**2, axis=0)).tolist(),
        "position_axis_max_abs_error_m": np.abs(position_error).max(axis=0).tolist(),
        "position_max_error_m": float(position_norm.max()),
        "velocity_rmse_mps": float(np.sqrt(np.mean(velocity_norm**2))),
        "velocity_axis_rmse_mps": np.sqrt(np.mean(velocity_error**2, axis=0)).tolist(),
        "velocity_axis_max_abs_error_mps": np.abs(velocity_error).max(axis=0).tolist(),
        "velocity_max_error_mps": float(velocity_norm.max()),
        "reference_mean_speed_mps": float(reference_speed.mean()),
        "reference_max_speed_mps": float(reference_speed.max()),
        "reference_max_acceleration_mps2": float(reference_acceleration_norm.max()),
        "reference_speed_at_max_acceleration_mps": float(
            np.linalg.norm(reference_velocity_all[max_acceleration_index])
        ),
        "reference_time_at_max_acceleration_s": float(
            reference_time_all[max_acceleration_index]
        ),
        "reference_axis_max_abs_acceleration_mps2": np.abs(reference_acceleration_all)
        .max(axis=0)
        .tolist(),
        "reference_max_feedforward_thrust_mps2": float(feedforward_thrust.max()),
        "reference_max_feedforward_thrust_to_weight": float(feedforward_thrust.max() / 9.81),
        "reference_min_body_z_world_z": float(body_z[:, 2].min()),
        "reference_max_tilt_deg": float(
            np.degrees(np.arccos(np.clip(body_z[:, 2], -1.0, 1.0))).max()
        ),
        # PX4 v1.13.3 Iris: four 5.84e-6 N/(rad/s)^2 rotors, 1.535 kg total mass.
        "iris_model_max_thrust_to_weight": float(
            4.0 * 5.84e-6 * 1100.0**2 / (1.535 * 9.81)
        ),
        # The command interface caps normalized thrust at 0.95; Iris maps that
        # to 100 + 1000 * 0.95 = 1050 rad/s per rotor.
        "iris_command_cap_thrust_to_weight": float(
            4.0 * 5.84e-6 * 1050.0**2 / (1.535 * 9.81)
        ),
    }
    if np.any(valid_control):
        control_command = control_command[valid_control]
        thrust = control_command[:, 0]
        body_rate = control_command[:, 1:4]
        metrics.update(
            {
                "command_max_thrust_normalized": float(thrust.max()),
                "command_thrust_upper_limit_fraction": float(np.mean(thrust >= 0.949)),
                "command_max_abs_bodyrate_xy_radps": float(np.abs(body_rate[:, :2]).max()),
                "command_max_abs_bodyrate_z_radps": float(np.abs(body_rate[:, 2]).max()),
                "command_bodyrate_xy_limit_fraction": float(
                    np.mean(np.any(np.abs(body_rate[:, :2]) >= 5.99, axis=1))
                ),
                "command_bodyrate_z_limit_fraction": float(np.mean(np.abs(body_rate[:, 2]) >= 3.99)),
            }
        )
    if samples.shape[1] >= 25:
        reference_quaternion = samples[:, 17:21]
        actual_quaternion = samples[:, 21:25]
        quaternion_dot = np.abs(np.sum(reference_quaternion * actual_quaternion, axis=1))
        attitude_error = 2.0 * np.arccos(np.clip(quaternion_dot, 0.0, 1.0))

        def body_x_heading(quaternion):
            w, x, y, z = quaternion.T
            body_x = np.stack(
                (
                    1.0 - 2.0 * (y**2 + z**2),
                    2.0 * (x * y + w * z),
                ),
                axis=1,
            )
            return np.arctan2(body_x[:, 1], body_x[:, 0]), np.linalg.norm(body_x, axis=1)

        reference_heading, reference_heading_norm = body_x_heading(reference_quaternion)
        actual_heading, actual_heading_norm = body_x_heading(actual_quaternion)
        heading_error = np.arctan2(
            np.sin(actual_heading - reference_heading),
            np.cos(actual_heading - reference_heading),
        )
        heading_valid = (reference_heading_norm > 0.1) & (actual_heading_norm > 0.1)
        tangent_heading = np.arctan2(samples[:, 8], samples[:, 7])
        reference_tangent_error = np.arctan2(
            np.sin(reference_heading - tangent_heading),
            np.cos(reference_heading - tangent_heading),
        )
        horizontal_speed = np.linalg.norm(samples[:, 7:9], axis=1)
        tangent_valid = heading_valid & (horizontal_speed > 0.1)
        high_speed_threshold = max(0.1, 0.8 * horizontal_speed.max())
        high_speed_valid = heading_valid & (horizontal_speed >= high_speed_threshold)
        metrics.update(
            {
                "attitude_rmse_deg": float(np.degrees(np.sqrt(np.mean(attitude_error**2)))),
                "attitude_max_error_deg": float(np.degrees(attitude_error.max())),
                "heading_rmse_deg": float(
                    np.degrees(np.sqrt(np.mean(heading_error[heading_valid] ** 2)))
                ),
                "heading_max_error_deg": float(
                    np.degrees(np.abs(heading_error[heading_valid]).max())
                ),
                "reference_tangent_heading_rmse_deg": float(
                    np.degrees(
                        np.sqrt(np.mean(reference_tangent_error[tangent_valid] ** 2))
                    )
                ),
                "reference_tangent_heading_max_error_deg": float(
                    np.degrees(np.abs(reference_tangent_error[tangent_valid]).max())
                ),
                "heading_high_speed_rmse_deg": float(
                    np.degrees(np.sqrt(np.mean(heading_error[high_speed_valid] ** 2)))
                ),
                "heading_high_speed_max_error_deg": float(
                    np.degrees(np.abs(heading_error[high_speed_valid]).max())
                ),
                "reference_tangent_heading_high_speed_rmse_deg": float(
                    np.degrees(
                        np.sqrt(np.mean(reference_tangent_error[high_speed_valid] ** 2))
                    )
                ),
                "reference_tangent_heading_high_speed_max_error_deg": float(
                    np.degrees(np.abs(reference_tangent_error[high_speed_valid]).max())
                ),
                "heading_high_speed_threshold_mps": float(high_speed_threshold),
            }
        )
    return metrics


def write_results(output, samples, metrics, arguments, trajectory_metadata=None):
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = output.with_suffix(".csv")
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "stamp",
                "ref_x",
                "ref_y",
                "ref_z",
                "actual_x",
                "actual_y",
                "actual_z",
                "ref_vx",
                "ref_vy",
                "ref_vz",
                "actual_vx",
                "actual_vy",
                "actual_vz",
                "command_thrust_normalized",
                "command_bodyrate_x_radps",
                "command_bodyrate_y_radps",
                "command_bodyrate_z_radps",
                "ref_qw",
                "ref_qx",
                "ref_qy",
                "ref_qz",
                "actual_qw",
                "actual_qx",
                "actual_qy",
                "actual_qz",
            ]
        )
        writer.writerows(samples.tolist())

    report = dict(metrics)
    report["trajectory"] = arguments.trajectory
    if arguments.trajectory == "poly":
        report["requested_peak_speed_mps"] = arguments.poly_peak_speed
    if trajectory_metadata is not None:
        report.update(trajectory_metadata)
    report["warmup_s"] = arguments.warmup
    report["csv"] = str(csv_path)
    with output.open("w") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory",
        choices=("horizontal-circle", "poly"),
        required=True,
    )
    parser.add_argument("--text-duration", type=float, default=27.0)
    parser.add_argument("--poly-peak-speed", type=float, default=3.0)
    parser.add_argument("--trial-duration", type=float)
    parser.add_argument("--warmup", type=float, default=0.5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world-velocity", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ommpc_sitl_test_runner")
    runner = TrialRunner(not args.world_velocity)
    wait_until(
        lambda: runner.state() is not None and runner.state().connected,
        30.0,
        "MAVROS connection",
    )
    wait_until(lambda: runner.altitude() is not None, 15.0, "local odometry")

    rospy.wait_for_service("/mavros/set_mode", timeout=15.0)
    rospy.wait_for_service("/mavros/param/set", timeout=15.0)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    set_param = rospy.ServiceProxy("/mavros/param/set", ParamSet)
    for parameter_name in ("COM_RC_IN_MODE", "COM_RCL_EXCEPT"):
        response = set_param(
            param_id=parameter_name,
            value=ParamValue(integer=4, real=0.0),
        )
        if not response.success:
            raise RuntimeError("failed to set PX4 parameter {}".format(parameter_name))
    dynamic = DynamicReconfigureClient("/ommpc_controller", timeout=15.0)
    dynamic.update_configuration(
        {"command_or_hover": False, "land_enabled": False, "takeoff_enabled": False}
    )

    rospy.sleep(2.0)
    for _ in range(8):
        set_mode(base_mode=0, custom_mode="OFFBOARD")
        try:
            wait_until(lambda: runner.state().mode == "OFFBOARD", 1.0, "OFFBOARD mode")
            break
        except RuntimeError:
            pass
    else:
        raise RuntimeError("PX4 rejected OFFBOARD mode")

    dynamic.update_configuration({"takeoff_enabled": True})
    wait_until(lambda: runner.state().armed, 15.0, "arming")
    takeoff_height = rospy.get_param("/ommpc_controller/takeoff_height")
    wait_until(lambda: runner.altitude() >= takeoff_height - 0.04, 45.0, "takeoff altitude")
    dynamic.update_configuration({"takeoff_enabled": False})
    rospy.sleep(3.0)

    trajectory_publisher = rospy.Publisher(
        "/drone_0_planning/trajectory", PolyTraj, queue_size=1, latch=True
    )
    trajectory_metadata = None
    if args.trajectory == "horizontal-circle":
        start = rospy.Time.now().to_sec()
        duration = args.text_duration
        runner.begin_recording(start, duration)
        dynamic.update_configuration({"command_or_hover": True})
        trajectory_finished = lambda: rospy.Time.now().to_sec() >= start + duration
    else:
        start_time = rospy.Time.now() + rospy.Duration(1.0)
        drag_enabled = rospy.get_param("/ommpc_controller/drag_compensation/enable")
        horizontal_drag = rospy.get_param(
            "/ommpc_controller/drag_compensation/coefficients/horizontal"
        ) if drag_enabled else 0.0
        vertical_drag = rospy.get_param(
            "/ommpc_controller/drag_compensation/coefficients/vertical"
        ) if drag_enabled else 0.0
        drag_speed_coefficient = rospy.get_param(
            "/ommpc_controller/drag_compensation/speed_coefficient"
        ) if drag_enabled else 0.0
        message, duration, trajectory_metadata = make_polynomial_message(
            start_time,
            args.poly_peak_speed,
            takeoff_height,
            rospy.get_param("/ommpc_controller/drag_compensation/mass"),
            horizontal_drag,
            vertical_drag,
            drag_speed_coefficient,
            rospy.get_param(
                "/ommpc_controller/sitl_test/max_reference_thrust_to_weight"
            ),
        )
        if args.trial_duration is not None:
            duration = min(duration, args.trial_duration)
            trajectory_metadata["executed_duration_s"] = duration
        trajectory_publisher.publish(message)
        dynamic.update_configuration({"command_or_hover": True})
        runner.begin_recording(start_time.to_sec(), duration)
        trajectory_finished = lambda: rospy.Time.now() >= start_time + rospy.Duration(duration)

    wait_until(trajectory_finished, duration + 5.0, "trajectory completion")
    dynamic.update_configuration({"command_or_hover": False})
    rospy.sleep(0.5)

    samples = runner.samples()
    drag_enabled = rospy.get_param("/ommpc_controller/drag_compensation/enable")
    horizontal_drag = rospy.get_param(
        "/ommpc_controller/drag_compensation/coefficients/horizontal"
    ) if drag_enabled else 0.0
    vertical_drag = rospy.get_param(
        "/ommpc_controller/drag_compensation/coefficients/vertical"
    ) if drag_enabled else 0.0
    drag_speed_coefficient = rospy.get_param(
        "/ommpc_controller/drag_compensation/speed_coefficient"
    ) if drag_enabled else 0.0
    metrics = compute_metrics(
        samples,
        args.warmup,
        rospy.get_param("/ommpc_controller/drag_compensation/mass"),
        horizontal_drag,
        vertical_drag,
        drag_speed_coefficient,
    )
    metrics["thrust_model_estimation_enabled"] = rospy.get_param(
        "/ommpc_controller/thrust_model_estimation/enable"
    )
    write_results(args.output, samples, metrics, args, trajectory_metadata)


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
        rospy.logfatal(str(error))
        raise
