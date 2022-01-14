#!/usr/bin/env python3
import json
import uuid
import rclpy
import navibee_msgs
from rclpy import duration
from rclpy.handle import InvalidHandle

from rclpy.node import Node
from navibee_msgs.msg import Task, TaskStatus, Goal, RobotStatus

from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time

from enum import Enum
import copy

class RobotStatus(Node):
    def __init__(self):
        """Initialising RobotStatus Node"""
        super().__init__('Robot_status_node')
        self.get_logger().info(
            "Getting RobotStatus Battery %"
        )

        # Create subscriber for /navibee/robotstatus
        self.subscriber = self.create_subscription(
            RobotStatus,
            'navibee/robotstatus',
            self.robotstatus_callback,
            10
        )

    def robotstatus_callback(self, msg):
        self.get_logger().info('======================================')
        self.get_logger().info('Battery Status: ')
        self.get_logger().info('    Battery %: {}'.format(msg.battpercentage))\


class CreateRetrieveTask(Node):
    def __init__(self): 
        """Initialising CreateReturnTask Node"""
        super().__init__('Create_Return_Task')
        self.get_logger().info(
            "Creating Task and Retrieving Task Status"
        )

        # Create publisher for /navibee/robottask
        self.publisher = self.create_publisher(
            Task,
            'navibee/robottask',
            10
        )
        
        self.publisher1 = self.create_publisher(
            TaskStatus,
            'navibee/robottaskstatus',
            10
        )

        # Create subscriber for /navibee/TaskStatus
        self.subscriber = self.create_subscription(
            TaskStatus,
            'navibee/robottaskstatus',
            self.retrievetask_callback,
            10
        )
    
    def CreateTask(self, pose_name, pose):

            self.get_logger().info('************************')
            self.get_logger().info('Sending to goal: {}'.format(pose_name))

            # Create a new Goal
            new_goal = Goal()
            new_goal.mapverid = "12345"
            new_goal.positionname = pose_name
            new_goal.x = float(pose[0])
            new_goal.y = float(pose[1])
            new_goal.theta = float(pose[2])

            # Create a new Task
            new_task = Task()
            new_task.modificationtype = 'CREATE'
            new_task.skillset = 'GOTO'
            skillsetparam = {
                'type': pose[3],
                'precision': 'high',
                'tolerance': 0.5,
                'retries': 3
            }

            new_task.skillsetparam = json.dumps(skillsetparam)
            new_task.goal = new_goal
            new_task.abort = False
            new_task.jobid = '04aa2dc4-80af-4d51-8182-67d814a8d4e3'

            # Publish Task
            self.get_logger().info('Publishing Task: ')
            self.get_logger().info('    Task ID: {}'.format(new_task.jobid))
            self.get_logger().info('    SKillset: {}'.format(new_task.skillset))
            self.get_logger().info('    Goal: [{}, {}, {}]'.format(
                new_goal.x, new_goal.y, new_goal.theta
            ))
            
            self.publisher.publish(new_task)

            # Create and publish task status
            new_taskstatus = TaskStatus()
            new_taskstatus.skillset = 'GOTO'
            new_taskstatus.status = 'COMPLETED'
            new_taskstatus.jobid = '04aa2dc4-80af-4d51-8182-67d814a8d4e3'
            
            self.get_logger().info('Publishing Task Status: ')
            self.get_logger().info('    Task status: {}'.format(new_taskstatus.status))
            self.get_logger().info('    Job ID: {}'.format(new_taskstatus.jobid))

            self.publisher1.publish(new_taskstatus)


    def retrievetask_callback(self, msg):
        self.get_logger().info('====================================')
        self.get_logger.info('Received Task Status: ')
        self.get_logger().info('    Task ID: {}').format(msg.jobid)
        self.get_logger().info('    Skillset: {}'.format(msg.skillset))
        self.get_logger().info('    Status: {}'.format(msg.status))
        self.get_logger().info('    Error msg: {}'.format(msg.errmsg))

        self.task_status = msg.status
        print('Task status is: ', self.task_status)
       

class State(Enum):
    IDLE = 1
    QUEUING_TASK = 2
    EXECUTING_TASK = 3
    AWAITING_FOR_EXECUTION = 4
    AWAITING_TASK_COMPLETION = 5

class Robot(Node):
    def __init__(self):
        super().__init__('Robot_node')
        self.get_logger().info('Robot starting up ...') 

        # Subscribe to CreateRetrieveTask Node
        self.subscriber = self.create_subscription(
            TaskStatus,
            'navibee/robottaskstatus',
            self.robotnodestatus_callback,
            10
        )

        # Configure all goals
        self.goals = {}
        self.goals['home'] = ['home', 1.1, 1.1, 1.1, 'pose']
        self.goals['goalA'] = ['goalA', 2.2, 2.2, 2.2, 'pose']
        self.goals['goalB'] = ['goalB', 3.3, 3.3, 3.3, 'pose']
        self.goals['goalC'] = ['goalC', 4.4, 4.4, 4.4, 'pose']


        # Initialise time
        interval_min = 10
        duration = Duration(nanoseconds=10 * 6e10)
        self.last_task_request_time = self.get_clock().now() - duration

        # Initialise state
        self.state = State.IDLE
        self.last_task_state_change_time = self.get_clock().now()
        self.time_to_rest_nanosec = 1e9
        self.last_state = self.state

        # Initialise task groups
        self.task_to_do = []
        self.task_status = ""
        self.task_error_msg = ""
        self.pose_name = ''

        # Looping function
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def robotnodestatus_callback(self, msg):
        self.task_status = msg.status
    
    def timer_callback(self):
        self.get_logger().info('----------Callback every 0.5 secs---------------')
        if (self.state == State.IDLE):
            
            self.goals_duplicate = copy.deepcopy(self.goals)
            self.goals_duplicate.pop('home')
            
            # Ask for user input
            self.goalsDuplicateToString = ' '.join([str(item) for item in self.goals_duplicate])
            print('\nYou are at: HOME')
            print('\nChoose your goal: {}'.format(self.goalsDuplicateToString))
            selected_goal = input()
            print('Goal selected: ', selected_goal)
            if selected_goal in self.goals_duplicate:
                self.task_to_do = self.goals_duplicate[selected_goal]
                print('Task to do is: ', self.task_to_do)

            elapsed_time = self.get_clock().now() - self.last_task_state_change_time
            if (elapsed_time.nanoseconds > self.time_to_rest_nanosec):
                self.state = State.QUEUING_TASK

            self.get_logger().info('State: IDLE')
            self.get_logger().info('    Elapsed time: {}'.format(elapsed_time.nanoseconds/ 1e9))
            self.get_logger().info('    Time to wait: {}'.format(self.time_to_rest_nanosec/ 1e9))

        elif (self.state == State.QUEUING_TASK):
            self.state = State.EXECUTING_TASK

            self.get_logger().info('State: QUEUING_TASK')
        
        elif (self.state == State.EXECUTING_TASK):
            pose_name = self.task_to_do[0]
            pose = [self.task_to_do[1], self.task_to_do[2], self.task_to_do[3], self.task_to_do[4]]

            CreateRetrieveTask().CreateTask(pose_name, pose)

            self.state = State.AWAITING_FOR_EXECUTION
            self.get_logger().info('State: EXECUTING_TASK')
            self.get_logger().info('    Going to: {}'.format(pose_name))
            self.get_logger().info('    Coordinates: {}'.format(pose))
        
        elif (self.state == State.AWAITING_FOR_EXECUTION):
            # Wait for 2 seconds to ensure task is received by NaviBee by listening to taskstatus, then go to AWAITING_TASK_COMPLETION
            elapsed_time = self.get_clock().now() - self.last_task_state_change_time
            if (elapsed_time.nanoseconds > 2e9):
                self.state = State.AWAITING_TASK_COMPLETION

            self.get_logger().info('State: AWAITING_FOR_EXECUTION')
            self.get_logger().info(' Time elapsed: {}'. format(elapsed_time.nanoseconds/ 1e9))


        
        elif (self.state == State.AWAITING_TASK_COMPLETION):
            self.get_logger().info('State: AWAITING_TASK_COMPLETION')
            self.get_logger().info('    Task status: {}'.format(self.task_status))

            if (self.pose_name == 'home'):
                self.state = State.IDLE
                self.pose_name = ''
            
            elif (self.task_status == "FAILED"):
                print('\nFailed to reach destination.\n Heading HOME now ...')
                self.task_to_do.pop()
                self.task_to_do = self.goals['home']
                self.pose_name = 'home'
                self.state = State.QUEUING_TASK

            elif (self.task_status == "COMPLETED"):
                print('\nDestination Reached.')
                go_home = False
                while go_home == False:
                    print('User, please click \'y\' to verify: ')
                    user_input = input()
                    if (user_input == 'y'):
                        print('\'y\' entered, heading HOME now ...')
                        go_home = True
                        self.task_to_do.pop()
                        self.pose_name = 'home'
                        self.task_to_do = self.goals['home']
                        self.state = State.QUEUING_TASK
                    else:
                        continue
                
            
        
        if (self.state != self.last_state):
            self.last_task_state_change_time = self.get_clock().now()
            self.last_state = self.state
            


def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    rclpy.spin(robot)

    #Destroy the node explicitly
    robot.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
