import math

# Constants
MAX_SPEED = 4.0
MIN_SPEED = 2.0
MID_SPEED = (MAX_SPEED + MIN_SPEED) / 2.0
WAYPOINT_LOOKAHEAD = 2


def reward_function(params):
    # Extracting parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering_angle = abs(params['steering_angle'])
    speed = params['speed']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    # Initialize reward
    reward = 1.0

    # Calculate the direction of the center line based on the closest waypoints
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Look ahead to further waypoints
    if closest_waypoints[1] + WAYPOINT_LOOKAHEAD < len(waypoints):
        next_point = waypoints[closest_waypoints[1] + WAYPOINT_LOOKAHEAD]

    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])

    # Calculate the direction in which the agent is heading
    heading_direction = math.atan2(params["y"] - prev_point[1], params["x"] - prev_point[0])

    # Calculate the difference between the track direction and the heading direction
    direction_diff = abs(track_direction - heading_direction)

    # Penalize the reward if the difference in direction is large (agent is not following the track)
    direction_threshold = 5.0
    if direction_diff > direction_threshold:
        reward *= (1.0 - (direction_diff - direction_threshold) / 50.0)

    # Penalize high steering angles
    if steering_angle > 10:
        reward *= (1.0 - (steering_angle - 10.0) / 50.0)

    # Speed reward
    if steering_angle > 10:
        expected_speed = MIN_SPEED
    elif steering_angle > 5:
        expected_speed = MID_SPEED
    else:
        expected_speed = MAX_SPEED

    # Penalize deviation from expected speed
    speed_diff = abs(speed - expected_speed)
    if speed_diff > 0.2:
        reward *= (1.0 - (speed_diff / MAX_SPEED))

    # Encourage staying in the center of the track
    if distance_from_center <= track_width / 2.0:
        reward *= 1.2 * (1.0 - (2.0 * distance_from_center / track_width))

    # Encourage maintaining balance
    if all_wheels_on_track:
        reward += 1.0

    # Bonus for progress
    reward += 1.2 * (progress / 100.0)

    # Heavily penalize if the car goes off track
    if not all_wheels_on_track or distance_from_center > (track_width / 2.0):
        reward = 1e-3

    return float(reward)
