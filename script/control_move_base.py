#!/usr/bin/env python3

# ---- Object being demoed ---- #
from bot_container.multirobot_class_control_sample import bot_container

# ---- basic imports ---- #
import rospy
import random

# ---- Messages ---- #
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseActionResult



class multipoint_example():
    def __init__(self) -> None:
        """
        Generates and initializes each robot so that it can be utilized with the go_to_pts_in_list function.
        """
        
        rospy.init_node("control_move_base", log_level=rospy.DEBUG)    
        self.bot_list = list()

        self.num_bots = rospy.get_param("num_bots", int)

        for idx in range(self.num_bots):
            # A unique name prefix for each robot
            uid = f"robot_{idx}"

            rospy.loginfo(f"Creating bot called \"{uid}\" access")

            # create all publishers and subscribers for bot
            bot = bot_container(uid)

            # Stores the initial position of the bot
            bot.init_position = bot.current_pose.pose.pose.position
            
            # add the bot object to the list
            self.bot_list.append(bot)
            
        rospy.sleep(5)


    def gen_goal_msg(self, point:Point) -> PoseStamped:
        """Converts a Point msg to a PoseStamped msg for publishing to move_base

        Args:
            point (Point): Point for bot to go to

        Returns:
            PoseStamed: msg for robot to go to.
        """
        out = PoseStamped()
        
        out.pose.position = point
        
        # move_base requires valid quaternions for messages
        # This makes it a valid 0 degree quaternion
        out.pose.orientation.w = 1
        
        out.header.seq = 0
        out.header.frame_id = 'map'
        out.header.stamp = rospy.Time.now()
        
        return out


    def go_to_pts_in_list(self, list_of_pts:list):
        """Publishes all input points to robots in order of them being free

        Args:
            list_of_pts (list): a list of all points for the robots to go to (Point need to be a ROS Point)
        """
        # While there is more pts to go to
        while (len(list_of_pts) > 0) and not rospy.is_shutdown() :
            rospy.logdebug("Accessing all bots...")
            
            for bot in self.bot_list:
                # Ensures each bot is correct object. 
                # It shouldn't be possible so desired result is crashing in that case.
                robot = assert_bot_control(bot)
                
                # if  robot is doing nothing  then
                if not robot.action_being_done:
                    # This example has no error checking on what the previous action was
                    
                    goal_pt = list_of_pts.pop(0)
                    
                    rospy.loginfo(f"Publishing goal of ({goal_pt.x}, {goal_pt.y}) to {robot.prefix}")
                    
                    goal_msg = self.gen_goal_msg(goal_pt)
                    
                    robot.publish_goal(goal_msg)
            
            
            rospy.sleep(0.5)


    def return_home(self):
        """Sends robots back to their starting positions"""
        
        rospy.loginfo("Starting return home")
        
        for bot in self.bot_list:
            robot = assert_bot_control(bot)
            
            goal = self.gen_goal_msg(robot.init_position)
            
            # If its current task was interrupted
            if robot.action_being_done:
                rospy.loginfo(f"{robot.prefix} goal was interrupted")
            
            robot.publish_goal(goal)
                            


def assert_bot_control(obj) -> bot_container:
    """
    This is a wrapper that acts similarly to typecasting to bot_container.
    It functionally does nothing and is unnecessary to code operation. 
    It is useful for type hinting and auto-suggestion with a good IDE.  
    
    Args:
        obj : an object

    Returns:
        bot_container: the object as a bot_container
    """
    assert isinstance(obj, bot_container)
    return obj

               
def rand_point() -> Point:
    """Generates a random x & y for a Point object"""
    x = random.randint(-100, 100) / 10
    y = random.randint(-100, 100) / 10
    return Point(x, y, 0)


def example_list(num_bots:int) -> list():
    """Generates a list of points for"""
    pts = list()
    # This helps ensure there is enough points available to demo no matter hom many robots it is controlling
    while len(pts) < int(2.5 * num_bots):
        pts.append(rand_point())
    
    pts.insert(0, Point(-4, -8, 0))
    pts.insert(3, Point(-8, 4, 0))
    pts.insert(1, Point(7, 4, 0))
    pts.insert(2, Point(5, -7, 0))
    
    return pts



if __name__ == "__main__":
    print("Running")
    bot_example = multipoint_example()
    
    pts = example_list(bot_example.num_bots)
    
    bot_example.go_to_pts_in_list(pts)
    
    rospy.sleep(5)
    
    rospy.loginfo("interrupting with return home")
    
    bot_example.return_home()