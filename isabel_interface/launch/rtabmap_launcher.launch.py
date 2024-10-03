from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	dir = get_package_share_directory('isabel_interface')
	return LaunchDescription([
    	IncludeLaunchDescription(
        	PythonLaunchDescriptionSource(dir + '/launch/prueba.launch.py'),
        	launch_arguments={
            	'rviz': 'true',
            	'visual_odometry': 'false',
            	'frame_id': 'bodyy',
            	'subscribe_scan': 'false',
            	'subscribe_scan_cloud': 'true',
            	'depth': 'false',
            	'approx_sync': 'true',
            	'odom_topic': '/odom',
            	'scan_cloud_topic': '/tf_points2',
            	'args': "-d --RGBD/NeighborLinkRefining true --Reg/Strategy 1 --Grid/FlatObstacleDetected false --Grid/NormalsSegmentation true --Grid/MaxGroundHeight -0.5 --Grid/MinGroundHeight -2.0 --Grid/RangeMax 0.0 --Grid/MaxGroundAngle 15 --GridGlobal/Eroded false --Optimizer/Strategy 2 --Optimizer/PriorsIgnored false --Optimizer/Robust false --GTSAM/Optimizer 2 --Bayes/FullPredictionUpdate false --Grid/RayTracing true --GridGlobal/FullUpdate true --GridGlobal/UpdateError 0.005 --GridGlobal/ProbHit 0.9 --GridGlobal/ProbMiss 0.45 --Grid/NoiseFilteringRadius 0.5 --Grid/PreVoxelFiltering true --Reg/PoseOptimization true --Vis/MinInliers 25 --Grid/RangeMin 2.0",
# --Grid/MaxObstacleHeight 0.5
# --Grid/Sensor 1 --GridGlobal/UpdateError 0.1 --GridGlobal/ProbHit 0.9 --GridGlobal/ProbMiss 0.45",
            	'queue_size': '1000',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'pos_tracking_enabled': 'false',
                #'use_sim_time': 'true',
 
                'Grid/Incremental': 'True',
                'Grid/UnknownSpaceFilled': 'False',
                'Grid/ColorMapOccupancy': 'True',

}.items(),
    	),
	])

