import numpy as np
import cv2
from scipy.optimize import least_squares


# Define the bundle_adjustment function
def bundle_adjustment(points_3d, cameras, observations, intrinsics, initial_params):
    num_points = points_3d.shape[0]
    num_cameras = len(cameras)
    num_observations = len(observations)
    camera_indices = [obs['camera_id'] for obs in observations]
    point_indices = [obs['point_id'] for obs in observations]
    observations_2d = np.array([obs['measurement'] for obs in observations]).reshape(-1, 2)

    def fun(params):
        points = params[:3 * num_points].reshape((num_points, 3))
        poses = params[3 * num_points:].reshape((num_cameras, 6))
        proj_errors = []

        for i, obs in enumerate(observations):
            camera = cameras[camera_indices[i]]
            point = points[point_indices[i]]
            pose = poses[camera.id]

            # Compute projection
            R = cv2.Rodrigues(pose[:3])[0]
            t = pose[3:]
            projection = intrinsics.K @ (R @ point + t)
            projected_point = projection[:2] / projection[2]

            # Compute reprojection error
            proj_error = projected_point - obs.measurement
            proj_errors.extend(proj_error)

        return proj_errors

    # Set initial parameters
    initial_params = np.concatenate((points_3d.ravel(), initial_params))

    # Perform bundle adjustment
    optimized_params = least_squares(fun, initial_params, method='lm').x

    # Retrieve optimized camera poses and 3D points
    optimized_points = optimized_params[:3 * num_points].reshape((num_points, 3))
    optimized_poses = optimized_params[3 * num_points:].reshape((num_cameras, 6))

    for i, camera in enumerate(cameras):
        camera.pose.rotation = optimized_poses[i, :3]
        camera.pose.translation = optimized_poses[i, 3:]

    return optimized_points, cameras


# Example usage
def main():
    # Generate some example data
    num_points = 10
    num_cameras = 2
    num_observations = 20

    # Generate random 3D points
    points_3d = np.random.randn(num_points, 3)

    # Generate random camera poses
    cameras = []
    for i in range(num_cameras):
        rotation = cv2.Rodrigues(np.random.randn(3))[0]
        translation = np.random.randn(3)
        cameras.append({'rotation': rotation, 'translation': translation})

    # Generate random observations
    observations = []
    for i in range(num_observations):
        camera_id = np.random.randint(num_cameras)
        point_id = np.random.randint(num_points)
        camera = cameras[camera_id]
        point_3d = points_3d[point_id]
        projection = np.dot(camera['rotation'], point_3d) + camera['translation']
        measurement = projection[:2] / projection[2]
        measurement += np.random.randn(2) * 0.1  # Add some noise
        observations.append({'camera_id': camera_id, 'point_id': point_id, 'measurement': measurement})

    # Set initial parameters
    initial_params = np.random.randn(num_points * 3 + num_cameras * 6)

    # Set intrinsics parameters (modify based on your intrinsics model)
    fx = 500
    fy = 500
    cx = 320
    cy = 240
    intrinsics = np.array([fx, fy, cx, cy])

    # Perform bundle adjustment
    optimized_points, optimized_cameras = bundle_adjustment(points_3d, cameras, observations, intrinsics,
                                                            initial_params)

    # Print the optimized results
    print("Optimized 3D Points:")
    print(optimized_points)
    print("Optimized Camera Poses:")
    for i, camera in enumerate(optimized_cameras):
        print(f"Camera {i + 1} - Rotation: {camera['rotation']}, Translation: {camera['translation']}")


if __name__ == '__main__':
    main()
