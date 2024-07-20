import rclpy
import math
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from collections import deque


"""
/**
 * @class MazePose
 * @brief Store the position and orientation of the robot in the maze, keep track of the robot's position and to check if a position has been visited before.
 */
"""
class MazePose():
    # Param: To retrieve the x and y coordinates and the z and w components of the orientation.
    def __init__(self, poseStampedObj):
        self.x = poseStampedObj.pose.position.x
        self.y = poseStampedObj.pose.position.y
        self.Orz = poseStampedObj.pose.orientation.z
        self.Orw = poseStampedObj.pose.orientation.w
    
    # Converts a MazePose back into a PoseStamped object.
    # Params: prevDest The previous destination, navObj
    def getPoseStamped(self, prevDest, navObj):
        currDest = PoseStamped()
        currDest.pose.position.x = prevDest.pose.position.x
        currDest.pose.position.y = prevDest.pose.position.y
        currDest.pose.orientation.z = prevDest.pose.orientation.z
        currDest.pose.orientation.w = prevDest.pose.orientation.w
        currDest.header.stamp = navObj.get_clock().now().to_msg()
        currDest.header.frame_id = 'map'
        return currDest
    
    # Return a hash value based on the x and y coordinates.
    # Improve isVisited time complexity.
    def __hash__(self):
        return hash((self.x, self.y))

    # Checks if two MazePose instances are equal by comparing their x and y coordinates with a tolerance
    # with another MazePose instance to compare.
    # return True if the x and y coordinates of the two instances are close.
    # Purpose is to check if the position or node has been visited before.
    # Adjust tolerance if the turtlebot keeps visiting the same position in the maze.
    def __eq__(self, other):
        if isinstance(other, MazePose):
            # 1e-5 : 1x10^-5
            return math.isclose(self.x, other.x, rel_tol=1e-5) and math.isclose(self.y, other.y, rel_tol=1e-5)
        return False


"""
/**
 * @class InitialPoseMarker
 * @brief A class that marks the initial pose of the robot.
 */
"""
class InitialPoseMarker(Node):
    # Initialise the InitialPoseMarker object, subscribe to the 'pose' topic, and initialuse startPose
    def __init__(self):
        super().__init__('InitialPoseMarker')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, 'pose', self.update_pose, 10)
        self.startPose = PoseStamped()

    # Updates the startPose with the new pose message.
    def update_pose(self, msg):
        self.startPose.pose.position.x = msg.pose.pose.position.x
        self.startPose.pose.position.y = msg.pose.pose.position.y
        self.startPose.pose.position.z = msg.pose.pose.position.z
        self.startPose.pose.orientation.z = msg.pose.pose.orientation.z
        self.startPose.pose.orientation.w = msg.pose.pose.orientation.w

    # Gets the initial pose of the robot, destroy subscription, and returns the startPose
    def get_initial_pose(self):
        self.startPose.header.frame_id = 'map'
        self.destroy_subscription(self.subscription)
        x_position = self.startPose.pose.position.x
        y_position = self.startPose.pose.position.y
        print("Initial pose: {}, {}".format(x_position, y_position))
        return self.startPose
    

"""
/**
 * @brief Check if a position has been visited.
 * return True if the position is new, None otherwise.
 *
 * @param queue The queue of positions to visit.
 * @param visitedPosSet The set of visited positions.
 * @param currPos The current position.
 */
"""
def isVisited(queue, visitedPosSet, currPos, navObj):
    tempMazeObj = MazePose(currPos)
    if tempMazeObj not in visitedPosSet:
        print("New node found.")
        visitedPosSet.add(tempMazeObj)
        queue.append(currPos)  # Add to the end of the queue
        return True
    else:
        print("Node visited before.")
        return None


"""
/**
 * @brief Creates a new pose from the previous destination.
 *
 * @param prevDest The previous destination.
 * @param navObj Navigation object
 * @return The new pose.
 */
"""
def create_pose_from_previous_destination(prevDest, navObj):
    currDest = PoseStamped()
    currDest.pose.position.x = prevDest.pose.position.x
    currDest.pose.position.y = prevDest.pose.position.y
    currDest.pose.orientation.z = prevDest.pose.orientation.z
    currDest.pose.orientation.w = prevDest.pose.orientation.w
    currDest.header.stamp = navObj.get_clock().now().to_msg()
    currDest.header.frame_id = 'map'
    return currDest


"""
/**
 * @brief Follow path found.
 *
 * @param navObj Navigation object.
 * @param path The path to follow.
 */
"""
def follow_path(navObj, path):
    navObj.followPath(path)
    while not navObj.isTaskComplete():
        pass


"""
/**
 * @brief Exploration of maze using BFS
 *
 * @param queue The queue of positions to visit.
 * @param visitedPosSet The set of visited positions.
 * @param navObj Navigation object.
 */
"""
def traverse(queue, visitedPosSet, navObj):
    # If the queue is empty, the maze is assumed to be explored
    if not queue:
        print("No more nodes in queue.")
        return

    # Get the current node in the priority queue
    prevDest = queue[0]

    # Check all directions, iterating over the x and y axis, negative cases included
    for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:

        # Create a new goal in the direction
        currDest = create_pose_from_previous_destination(prevDest, navObj)

        # Update x and y coords
        currDest.pose.position.x += dx
        currDest.pose.position.y += dy

        # Check if the path exists and have not been visited
        path = navObj.getPath(prevDest, currDest)
        if path is not None and isVisited(queue, visitedPosSet, currDest, navObj) is not None:

            # Path exists to a space not visited before, add it to the queue
            print("Path found. Going from ({}, {}) to ({}, {})".format(prevDest.pose.position.x, prevDest.pose.position.y, currDest.pose.position.x, currDest.pose.position.y))
            follow_path(navObj, path)
            navObj.clearAllCostmaps()

    # Pop visited node
    queue.popleft()

    # Move on to the next node recursively if there are nodes left in the pq
    if queue:  
        traverse(queue, visitedPosSet, navObj)


def main():
    rclpy.init()

    # Initialize the navObj
    navObj = BasicNavigator()
    navObj.node_name = "NAV2API"

    # Get the initial pose
    pose_marker = InitialPoseMarker()
    initial_pose = pose_marker.get_initial_pose()
    initial_pose.header.stamp = navObj.get_clock().now().to_msg()
    navObj.setInitialPose(initial_pose)

    # Initialize the queue and set for BFS
    queue = deque([initial_pose])
    visitedPosSet = set([MazePose(initial_pose)])

    # Start BFS
    isComplete = traverse(queue, visitedPosSet, navObj)

    if isComplete:
        print("BFS maze exploration concluded")

if __name__ == '__main__':
    main()

