#!/usr/bin/env python3.8

################################# imports for GUI ############################################################
from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.widget import Widget
from kivymd.app import MDApp
from kivymd.uix.button import MDRaisedButton
from kivy.properties import StringProperty, ObjectProperty
from kivy.uix.togglebutton import ToggleButton
from kivy.uix.screenmanager import Screen, ScreenManager
import kivymd.icon_definitions
from kivymd.icon_definitions import md_icons
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.dropdown import DropDown
from kivymd.uix.menu import MDDropdownMenu
from kivy.lang import Builder
from kivy.clock import Clock
#from kivy.garden.cefpython import CEFBrowser, cefpython

#################################### imports for ROS and FSM ###############################################
from threading import *
import json
import uuid
import rclpy
from rclpy.node import Node
from navibee_msgs.msg import Task, TaskStatus, Goal, RobotStatus

from rclpy.duration import Duration

from enum import Enum
import copy
import yaml

########################################### GUI codes ######################################################

class TableScreen(Screen):
    Screen.program_paths = False
    table_no = StringProperty(None)
    message_1 = StringProperty()
    message_2 = StringProperty()
    message_3 = StringProperty()
    message_4 = StringProperty()
    message_5 = StringProperty()
    message_6 = StringProperty()
    
    def on_enter(self, *args):
        Screen.check_batt = False
        Screen.butt_go = ''
        Screen.home = ''
        Screen.table_no = ''
        Screen.to_home = None   
        Screen.state = ''
        Screen.home_reached = None
        Screen.program_paths = False
        Screen.pose_name = ''
        print('Screen.program_paths is: ', Screen.program_paths)
        robot.paths = []
        print(len(robot.goals)-1)

        if "Table 1" in robot.goals:
            position_1_coordinates = list(robot.goals['Table 1'][1:4])
            position_1_coordinates_to_string = ', '.join([str(elem) for elem in position_1_coordinates])
            self.message_1 = "                Table 1 \n  " 
            self.message_1 += '{' + position_1_coordinates_to_string + '}'
        else:
            self.message_1 = "            Table 1 \n  " 
            self.message_1 += '-' + ' Empty, no goal set ' + '-'

        # getting position 2
        if "Table 2" in robot.goals:
            position_2_coordinates = list(robot.goals['Table 2'][1:4])
            position_2_coordinates_to_string = ', '.join([str(elem) for elem in position_2_coordinates])
            self.message_2 = "                Table 2 \n  " 
            self.message_2 += '{' + position_2_coordinates_to_string + '}'
        else:
            self.message_2 = "            Table 2 \n  " 
            self.message_2 += '-' + ' Empty, no goal set ' + '-'

        # getting position 3
        if "Table 3" in robot.goals:
            position_3_coordinates = list(robot.goals['Table 3'][1:4])
            position_3_coordinates_to_string = ', '.join([str(elem) for elem in position_3_coordinates])
            self.message_3 = "                Table 3 \n  "  
            self.message_3 += '{' + position_3_coordinates_to_string + '}'
        else:
            self.message_3 = "            Table 3 \n  " 
            self.message_3 += '-' + ' Empty, no goal set ' + '-'

        # getting position 4
        if "Table 4" in robot.goals:
            position_4_coordinates = list(robot.goals['Table 4'][1:4])
            position_4_coordinates_to_string = ', '.join([str(elem) for elem in position_4_coordinates])
            self.message_4 = "                Table 4 \n  " 
            self.message_4 += '{' + position_4_coordinates_to_string + '}'
        else:
            self.message_4 = "            Table 4 \n  " 
            self.message_4 += '-' + ' Empty, no goal set ' + '-'

        # getting position 5
        if "Table 5" in robot.goals:
            position_5_coordinates = list(robot.goals['Table 5'][1:4])
            position_5_coordinates_to_string = ', '.join([str(elem) for elem in position_5_coordinates])
            self.message_5 = "                Table 5 \n  " 
            self.message_5 += '{' + position_5_coordinates_to_string + '}'
        else:
            self.message_5 = "            Table 5 \n  " 
            self.message_5 += '-' + ' Empty, no goal set ' + '-'

        # getting position 6
        if "Table 6" in robot.goals:
            position_6_coordinates = list(robot.goals['Table 6'][1:4])
            position_6_coordinates_to_string = ', '.join([str(elem) for elem in position_6_coordinates])
            self.message_6 = "                Table 6 \n  " 
            self.message_6 += '{' + position_6_coordinates_to_string + '}'
        else:
            self.message_6 = "            Table 6 \n  " 
            self.message_6 += '-' + ' Empty, no goal set ' + '-'

    def check_which_table(self):
        if self.ids.butt1.state == 'down':
            print('Table 1 has been pressed')
        elif self.ids.butt2.state == 'down':
            print('Table 2 has been pressed')
        elif self.ids.butt3.state == 'down':
            print('Table 3 has been pressed')
        elif self.ids.butt4.state == 'down':
            print('Table 4 has been pressed')
        elif self.ids.butt5.state == 'down':
            print('Table 5 has been pressed')
        elif self.ids.butt6.state == 'down':
            print('Table 6 has been pressed')
    
    def go_to_table(self):
        if self.ids.butt1.state == 'down' and 'Empty' not in self.message_1:
            Screen.table_no = 'Table 1'
            print('Navigating to', Screen.table_no)
            
        elif self.ids.butt2.state == 'down' and 'Empty' not in self.message_2:
            Screen.table_no = 'Table 2'
            print('Navigating to', Screen.table_no)

        elif self.ids.butt3.state == 'down' and 'Empty' not in self.message_3:
            Screen.table_no = 'Table 3'
            print('Navigating to', Screen.table_no)

        elif self.ids.butt4.state == 'down' and 'Empty' not in self.message_4:
            Screen.table_no = 'Table 4'
            print('Navigating to', Screen.table_no )

        elif self.ids.butt5.state == 'down' and 'Empty' not in self.message_5:
            Screen.table_no = 'Table 5'
            print('Navigating to', Screen.table_no)

        elif self.ids.butt6.state == 'down' and 'Empty' not in self.message_6:
            Screen.table_no = 'Table 6'
            print('Navigating to', Screen.table_no)

        else:
            print('Error! Invalid Goal!')
            self.ids.tl1.text = "\n That Goal is not set!"
            return False

    def on_leave(self, *args):
        self.ids.tl1.text = ""

    pass

class MoreTableScreen(Screen):
    message_7 = StringProperty()
    message_8 = StringProperty()
    message_9 = StringProperty()
    message_10 = StringProperty()
    message_11 = StringProperty()
    message_12 = StringProperty()

    def on_enter(self, *args):

        # getting position 7
        if "Table 7" in robot.goals:
            position_7_coordinates = list(robot.goals['Table 7'][1:4])
            position_7_coordinates_to_string = ', '.join([str(elem) for elem in position_7_coordinates])
            self.message_7 = "                Table 7 \n  " 
            self.message_7 += '{' + position_7_coordinates_to_string + '}'
        else:
            self.message_7 = "            Table 7 \n  " 
            self.message_7 += '-' + ' Empty, no goal set ' + '-'

        # getting position 8
        if "Table 8" in robot.goals:
            position_8_coordinates = list(robot.goals['Table 8'][1:4])
            position_8_coordinates_to_string = ', '.join([str(elem) for elem in position_8_coordinates])
            self.message_8 = "                Table 8 \n  " 
            self.message_8 += '{' + position_8_coordinates_to_string + '}'
        else:
            self.message_8 = "            Table 8 \n  " 
            self.message_8 += '-' + ' Empty, no goal set ' + '-'

        # getting position 9
        if "Table 9" in robot.goals:
            position_9_coordinates = list(robot.goals['Table 9'][1:4])
            position_9_coordinates_to_string = ', '.join([str(elem) for elem in position_9_coordinates])
            self.message_9 = "                Table 9 \n  " 
            self.message_9 += '{' + position_9_coordinates_to_string + '}'
        else:
            self.message_9 = "            Table 9 \n  " 
            self.message_9 += '-' + ' Empty, no goal set ' + '-'

        # getting position 10
        if "Table 10" in robot.goals:
            position_10_coordinates = list(robot.goals['Table 10'][1:4])
            position_10_coordinates_to_string = ', '.join([str(elem) for elem in position_10_coordinates])
            self.message_10 = "                Table 10 \n  " 
            self.message_10 += '{' + position_10_coordinates_to_string + '}'
        else:
            self.message_10 = "            Table 10 \n  " 
            self.message_10 += '-' + ' Empty, no goal set ' + '-'

        # getting position 11
        if "Table 11" in robot.goals:
            position_11_coordinates = list(robot.goals['Table 11'][1:4])
            position_11_coordinates_to_string = ', '.join([str(elem) for elem in position_11_coordinates])
            self.message_11 = "                Table 11 \n  " 
            self.message_11 += '{' + position_11_coordinates_to_string + '}'
        else:
            self.message_11 = "            Table 11 \n  " 
            self.message_11 += '-' + ' Empty, no goal set ' + '-'

        # getting position 12
        if "Table 12" in robot.goals:
            position_12_coordinates = list(robot.goals['Table 12'][1:4])
            position_12_coordinates_to_string = ', '.join([str(elem) for elem in position_12_coordinates])
            self.message_12 = "                Table 12 \n  " 
            self.message_12 += '{' + position_12_coordinates_to_string + '}'
        else:
            self.message_12 = "            Table 12 \n  " 
            self.message_12 += '-' + ' Empty, no goal set ' + '-'

    def check_which_table(self):
        if self.ids.butt7.state == 'down':
            print('Table 7 has been pressed')
        elif self.ids.butt8.state == 'down':
            print('Table 8 has been pressed')
        elif self.ids.butt9.state == 'down':
            print('Table 9 has been pressed')
        elif self.ids.butt10.state == 'down':
            print('Table 10 has been pressed')
        elif self.ids.butt11.state == 'down':
            print('Table 11 has been pressed')
        elif self.ids.butt12.state == 'down':
            print('Table 12 has been pressed')

    def go_to_table(self):
        if self.ids.butt7.state == 'down' and 'Empty' not in self.message_7:
            Screen.table_no = 'Table 7'
            print('Navigating to', Screen.table_no)
        elif self.ids.butt8.state == 'down'and 'Empty' not in self.message_8: 
            Screen.table_no = 'Table 8'
            print('Navigating to', Screen.table_no)
        elif self.ids.butt9.state == 'down' and 'Empty' not in self.message_9:
            Screen.table_no = 'Table 9'
            print('Navigating to', Screen.table_no)
        elif self.ids.butt10.state == 'down' and 'Empty' not in self.message_10:
            Screen.table_no = 'Table 10'
            print('Navigating to', Screen.table_no)
        elif self.ids.butt11.state == 'down' and 'Empty' not in self.message_11:
            Screen.table_no = 'Table 11'
            print('Navigating to', Screen.table_no)
        elif self.ids.butt12.state == 'down' and 'Empty' not in self.message_12:
            Screen.table_no = 'Table 12'
            print('Navigating to', Screen.table_no)

        else:
            print('Error! Invalid Goal!')
            self.ids.mtl1.text = "That Goal is not set!"
            return False   
    
    def on_leave(self, *args):
        self.ids.mtl1.text = ""

class MultiPathScreen(Screen):
    Screen.last_goal_in_path = False

    def check_valid(self):
        print(robot.paths)
        if self.ids.dpbtn1.text == 'Select' or self.ids.dpbtn2.text == 'Select' or self.ids.dpbtn3.text == 'Select' or self.ids.dpbtn4.text == 'Select' or self.ids.dpbtn5.text == 'Select':
            return False

        elif len(robot.paths) == 0:
            return False

        else:
            return True

    def queue_task(self):
        if self.ids.dpbtn1.text != '--' and self.ids.dpbtn1.text in robot.goals:
            first_path = {}
            first_path[self.ids.dpbtn1.text] = robot.goals[self.ids.dpbtn1.text]
            robot.paths.append(first_path)
            
        if self.ids.dpbtn2.text != '--' and self.ids.dpbtn2.text in robot.goals:
            second_path = {}
            second_path[self.ids.dpbtn2.text] = robot.goals[self.ids.dpbtn2.text]
            robot.paths.append(second_path)

        if self.ids.dpbtn3.text != '--' and self.ids.dpbtn3.text in robot.goals: 
            third_path = {}
            third_path[self.ids.dpbtn3.text] = robot.goals[self.ids.dpbtn3.text]
            robot.paths.append(third_path)

        if self.ids.dpbtn4.text != '--' and self.ids.dpbtn4.text in robot.goals:
            fourth_path = {}
            fourth_path[self.ids.dpbtn4.text] = robot.goals[self.ids.dpbtn4.text]
            robot.paths.append(fourth_path)

        if self.ids.dpbtn5.text != '--' and self.ids.dpbtn5.text in robot.goals:
            fifth_path = {}
            fifth_path[self.ids.dpbtn5.text] = robot.goals[self.ids.dpbtn5.text]
            robot.paths.append(fifth_path)

        print('Robot paths are: ', robot.paths)
        print('Robot path length: ', len(robot.paths))
        Screen.program_paths = True

    def show_error(self):
        self.ids.mpl1.text = "\nError! Please check your settings again."

    def on_leave(self, *args):
        self.ids.mpl1.text = ""
    
class NavigatingScreen(Screen):
    Screen.nav_succ = None
    Screen.cancel_navigation = None
    Screen.event = ObjectProperty()

    butt_go = StringProperty(None)
    
    def on_enter(self):
        Screen.nav_succ = None

        if Screen.program_paths:
            Screen.event = Clock.schedule_interval(self.callback2, 0.5)

        elif Screen.program_paths == False:
            print('button pressed')
            Screen.butt_go = 'yes'
            self.ids.nl.text = 'Navigating to ' + str(Screen.table_no) + " ..."
            Clock.schedule_interval(self.callback1, 0.5)

    
    def callback1(self, dt):
        if Screen.nav_succ == True:
            self.manager.current = 'succ'
            Screen.nav_succ = None

        elif Screen.nav_succ == False:
            self.manager.current = 'fail'
            Screen.nav_succ = None
            
        elif Screen.state == 'home':
            self.manager.current = 'to_home'

    def callback2(self, dt):
        if Screen.pose_name != '':
            self.ids.nl.text = 'Navigating to ' + Screen.pose_name + " ..."

            if Screen.nav_succ == True:
                print('second if statement entered')
                self.manager.current = 'succ'
                Screen.nav_succ = None
                Clock.unschedule(Screen.event)

            elif Screen.nav_succ == False:
                self.manager.current = 'fail'
                Screen.nav_succ = None
                Clock.unschedule(Screen.event)
                
            elif Screen.state == 'home':
                self.manager.current = 'to_home'
                Clock.unschedule(Screen.event)

    def cancel_navigation(self):
        print('Navigation cancelled')
        Screen.cancel_navigation = True
        self.manager.current = 'to_home'
    pass


class SuccessfulScreen(Screen):
    Screen.butt_go = ''
    Screen.next_goal = False

    def on_pre_enter(self, *args):
        if Screen.program_paths == False:
            self.ids.sb2.disabled = True

    def set_next_goal(self):

        if Screen.last_goal_in_path == False:
            Screen.next_goal = True
            self.manager.current = 'navigating'

        else:
            self.ids.sl3.text = 'This is the last goal in the path. Please return home.'
            self.ids.sb2.disabled = True


    def show_error(self):
        self.ids.sl3.text = 'Next goal only available in "multiple paths" mode, please return home.'
        self.ids.sb2.disabled = True

    def on_leave(self, *args):
        self.ids.sl3.text = ''
        self.ids.sb2.disabled = False

    pass
 

class ToHomeScreen(Screen):

    def on_enter(self, *args):
        Screen.home_reached = None
        Screen.to_home = True
        print('navigating to home')
        self.ids.hl.text = 'Navigating to Home ...'
        Clock.schedule_interval(self.callback, 0.5)
    
    def callback(self, dt):
        if Screen.home_reached == True:
            Screen.state = 'table'
            Screen.to_home = False
            self.manager.current = 'table'
            Screen.home_reached = False        
            
    pass

class FailScreen(Screen):
    Screen.butt_go = ''
    Screen.next_goal = False

    def on_pre_enter(self, *args):
        if Screen.program_paths == False:
            self.ids.fb2.disabled = True

    def set_next_goal(self):

        if Screen.last_goal_in_path == False:
            Screen.next_goal = True
            self.manager.current = 'navigating'

        else:
            self.ids.fl2.text = 'You have reached the last goal in the path. Please return home.'
            self.ids.fb2.disabled = True

    def show_error(self):
        self.ids.fl2.text = 'Next goal only available in "multiple paths" mode, please return home.'

    def on_leave(self, *args):
        self.ids.fl2.text = ''
        self.ids.fb2.disabled = False

    pass

class BatteryScreen(Screen):

    def on_pre_enter(self, *args):
        Screen.check_batt = True
        Screen.batt_per = None
        Clock.schedule_interval(self.callback, 0.5)

    def callback(self, dt):
        if Screen.batt_per != None:
            self.ids.bl.text = str(round(Screen.batt_per,1)) + '%'

    def on_leave(self, *args):
        Screen.state = 'table'
        Screen.check_batt = False
        Screen.batt_per = None
        return super().on_leave(*args)
        
    pass

class WindowManager(ScreenManager):
    pass

class MainApp(MDApp):

    def build(self):
        
        print("Number of goals: ", len(robot.goals)-1)
        self.kv= Builder.load_file("main.kv")
        return self.kv
    

########################################### End of GUI codes #####################################################

##########################################  FSM and ROS codes ####################################################

class RobotBattery(Node):

    # Node in charge of getting Robot Battery
    def __init__(self):
        """Initialising Robot Battery Node"""
        super().__init__('Robot_battery_node')
        self.get_logger().info(
            "Getting Robot Battery %"
        )

        # Create subscriber for /navibee/robotstatus
        self.subscriber1 = self.create_subscription(
            RobotStatus,
            'navibee/robotstatus',
            self.robotstatus_callback,
            10
        )
        self.batt_per = 0.0

    def robotstatus_callback(self, msg):
        self.get_logger().info('======================================')
        self.get_logger().info('Battery Status: ')
        self.get_logger().info('    Battery %: {}'.format(msg.battpercentage))
        self.batt_per = msg.battpercentage

        # Store battery percentage in a Screen variable to display in the UI
        Screen.batt_per = self.batt_per


class TaskCreation(Node):
    
    # Node in charge of creating Task and publishing Task
    def __init__(self): 
        """Initialising TaskCreation Node"""
        super().__init__('Task_Creation_Node')
        self.get_logger().info(
            "Creating Task"
        )

        # Create publisher for /navibee/robottask
        self.publisher = self.create_publisher(
            Task,
            'navibee/robottask',
            10
        )
        
        # Uncomment this portion to test without robot, commnet it when testing with robot:
        self.publisher1 = self.create_publisher(
            TaskStatus,
            'navibee/robottaskstatus',
            10
         )
    
    def create_task(self, pose_name, pose):

        # Function to create a new task
        self.get_logger().info('************************')
        self.get_logger().info('Sending to goal: {}'.format(pose_name))

        # Create a new Goal
        new_goal = Goal()
        new_goal.mapverid = "0c41fc57-3c16-471e-95ad-e10087a4f6ef"
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
            'retries': 4
        }

        new_task.skillsetparam = json.dumps(skillsetparam)
        new_task.goal = new_goal
        new_task.abort = False
        new_task.jobid = str(uuid.uuid4())

        # Publish Task
        self.get_logger().info('Publishing Task: ')
        self.get_logger().info('    Task ID: {}'.format(new_task.jobid))
        self.get_logger().info('    SKillset: {}'.format(new_task.skillset))
        self.get_logger().info('    Goal: [{}, {}, {}]'.format(
            new_goal.x, new_goal.y, new_goal.theta
        ))
        
        Robot.new_task = new_task
        
        self.publisher.publish(new_task)

        # Create and publish task status, uncomment this portion to test without robot, comment it when testing with robot:
        new_taskstatus = TaskStatus()
        new_taskstatus.skillset = 'GOTO'
        new_taskstatus.status = 'COMPLETED'
        new_taskstatus.jobid = '04aa2dc4-80af-4d51-8182-67d814a8d4e3'
        
        self.get_logger().info('Publishing Task Status: ')
        self.get_logger().info('    Task status: {}'.format(new_taskstatus.status))
        self.get_logger().info('    Job ID: {}'.format(new_taskstatus.jobid))

        self.publisher1.publish(new_taskstatus)

    def cancel_task(self, task_to_cancel):

        # Function to cancel the task that was published
        task_to_cancel.modificationtype = 'CANCEL'

        # Publish Task
        self.get_logger().info('Cancelling Task: ')
        self.get_logger().info('    Task ID: {}'.format(task_to_cancel.jobid))
        self.get_logger().info('    SKillset: {}'.format(task_to_cancel.skillset))
        
        self.publisher.publish(task_to_cancel)


class State(Enum):

    # For FSM states
    IDLE = 1
    CREATE_TASK = 2
    AWAITING_FOR_EXECUTION = 3
    AWAITING_TASK_COMPLETION = 4
    STATUS_OF_TASK = 5
    CHECK_BATT = 6
    MULTIPLE_TASKS = 7


class Robot(Node):

    # FSM that will be running together with GUI at the same time throughout the program
    def __init__(self):

        super().__init__('Robot_Node')
        self.get_logger().info('Robot starting up ...') 

        # Subscribe to TaskCreation Node
        self.subscriber = self.create_subscription(
            TaskStatus,
            'navibee/robottaskstatus',
            self.robottaskstatus_callback,
            10
        )
        
        # Open YAML file which contains the goals
        # Open YAML file and read comments, the goals have some specific configuration 
        with open('goals.yaml') as file:
            goals = yaml.load(file, Loader=yaml.FullLoader)

        print('Goals are: ', goals)
        self.goals = goals
        self.paths = []

        # Initialise time
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
        self.i = 0
        self.last_goal_in_path = False

        # Looping function
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.goals_duplicate = copy.deepcopy(self.goals)
        self.goals_duplicate.pop('home')
        self.goalsDuplicateToString = ' '.join([str(item) for item in self.goals_duplicate])

        # self.home_again will be set to True when robot is navigating back home. Initialise as False at first
        self.home_again = False

        # Create an instance of TaskCreation and RobotBattery node, store it as a variable under self
        self.create_task_node = TaskCreation()
        self.batt_status_node = RobotBattery()

    def robottaskstatus_callback(self, msg):

        # This will be printed when self.create_task_node is spinned
        self.get_logger().info('====================================')
        self.get_logger().info('Received Task Status: ')
        self.get_logger().info('    Task ID: {}'.format(msg.jobid))
        self.get_logger().info('    Skillset: {}'.format(msg.skillset))
        self.get_logger().info('    Status: {}'.format(msg.status))
        self.get_logger().info('    Error msg: {}'.format(msg.errmsg))

        self.task_status = msg.status
    

    def timer_callback(self):

        # FSM checks the state every 1s
        if (self.state == State.IDLE):

            print('\nYou are at: HOME')
            print('\nChoose your goal: {}'.format(self.goalsDuplicateToString))
            self.get_logger().info('State: IDLE')

            if Screen.check_batt == True:
                self.state = State.CHECK_BATT

            elif self.home_again == True:
                print('home again is true!')
                print('Welcome back to HOME')
                Screen.home_reached = True
                self.home_again = False
                Screen.butt_go = 'no'
                self.state = State.IDLE
                self.i = 0
                self.paths = []

                if Screen.check_batt == True:
                    self.state = State.CHECK_BATT

            elif Screen.program_paths:
                self.state = State.MULTIPLE_TASKS

            elif Screen.butt_go == 'yes' and self.home_again == False:
                if Screen.table_no == 'Table 1':
                    self.task_to_do = self.goals_duplicate['Table 1']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 2':
                    self.task_to_do = self.goals_duplicate['Table 2']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 3':
                    self.task_to_do = self.goals_duplicate['Table 3']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 4':
                    self.task_to_do = self.goals_duplicate['Table 4']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 5':
                    self.task_to_do = self.goals_duplicate['Table 5']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 6':
                    self.task_to_do = self.goals_duplicate['Table 6']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 7':
                    self.task_to_do = self.goals_duplicate['Table 7']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 8':
                    self.task_to_do = self.goals_duplicate['Table 8']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 10':
                    self.task_to_do = self.goals_duplicate['Table 10']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 11':
                    self.task_to_do = self.goals_duplicate['Table 11']
                    print('Task to do is: ', self.task_to_do)
                if Screen.table_no == 'Table 12':
                    self.task_to_do = self.goals_duplicate['Table 12']
                    print('Task to do is: ', self.task_to_do)

                elapsed_time = self.get_clock().now() - self.last_task_state_change_time
                self.get_logger().info('    Elapsed time: {}'.format(elapsed_time.nanoseconds/ 1e9))
                self.state = State.CREATE_TASK

        
        elif (self.state == State.CHECK_BATT):
            self.get_logger().info('State: CHECK_BATT')
            rclpy.spin_once(self.batt_status_node)
            self.get_logger().info('Battery Percentage: {}'.format(Screen.batt_per))
            if Screen.state == 'table':
                print('Going back to table screen')
                self.state = State.IDLE
        
        elif (self.state == State.CREATE_TASK):
            if Screen.cancel_navigation == True:
                Screen.cancel_navigation = False
                Screen.state = 'home'
                self.home_again = True
                self.pose_name = 'home'
                self.task_to_do = self.goals['home']
                self.state = State.CREATE_TASK
            
            else:
                pose_name = self.task_to_do[0]
                pose = [self.task_to_do[1], self.task_to_do[2], self.task_to_do[3], self.task_to_do[4]]

                self.create_task_node.create_task(pose_name, pose)

                self.state = State.AWAITING_FOR_EXECUTION
                self.get_logger().info('State: EXECUTING_TASK')
                self.get_logger().info('    Going to: {}'.format(pose_name))
                self.get_logger().info('    Coordinates: {}'.format(pose))

        
        elif (self.state == State.MULTIPLE_TASKS):
            Screen.next_goal = False
            print("next.goal is: ", Screen.next_goal)

            if self.i==len(robot.paths)-1:
                print('This is the last goal in the path')
                Screen.last_goal_in_path = True
                print('Screen.last_goal_in_path is: ', Screen.last_goal_in_path)
            
            if self.i<len(robot.paths):
                pose_key = list(robot.paths[self.i].keys())[0]
                pose_keyToStr = ''.join([str(elem) for elem in pose_key])
                path = robot.paths[self.i][pose_keyToStr]
                self.pose_name = path[0]
                pose = [path[1], path[2], path[3], path[4]]
                print('length of paths: ', len(robot.paths))
                print('pose_name is: ', self.pose_name)
                Screen.pose_name = self.pose_name 
                print('pose is: ', pose)

                self.create_task_node.create_task(self.pose_name, pose)
                self.i += 1
                self.state = State.AWAITING_FOR_EXECUTION

            else:
                print('Error! i is more than length of path')

            self.get_logger().info('State: MULTIPLE_TASKS')
            self.get_logger().info('   Going to: {}'.format(self.pose_name))


        elif (self.state == State.AWAITING_FOR_EXECUTION):
            if Screen.cancel_navigation == True:
                self.create_task_node.cancel_task(Robot.new_task)
                Screen.state = 'home'
                Screen.cancel_navigation = False
                self.home_again = True
                self.pose_name = 'home'
                self.task_to_do = self.goals['home']
                self.state = State.CREATE_TASK

            else: 
                # Wait for 2 seconds to ensure task is received by NaviBee by listening to taskstatus, then go to AWAITING_TASK_COMPLETION
                elapsed_time = self.get_clock().now() - self.last_task_state_change_time
                if (elapsed_time.nanoseconds > 2e9):
                    self.state = State.AWAITING_TASK_COMPLETION

                self.get_logger().info('State: AWAITING_FOR_EXECUTION')
                self.get_logger().info(' Time elapsed: {}'. format(elapsed_time.nanoseconds/ 1e9))


        
        elif (self.state == State.AWAITING_TASK_COMPLETION):
            self.get_logger().info('State: AWAITING_TASK_COMPLETION')
            self.get_logger().info('    Task status: {}'.format(self.task_status))

            if Screen.cancel_navigation == True:
                self.create_task_node.cancel_task(Robot.new_task)
                Screen.state = 'home'
                Screen.cancel_navigation = False
                self.home_again = True
                self.pose_name = 'home'
                self.task_to_do = self.goals['home']
                self.state = State.CREATE_TASK
            
            elif (self.task_status == 'FAILED' and self.home_again == True): 
                self.state = State.CREATE_TASK

            elif (self.task_status == "FAILED"):
                Screen.nav_succ = False
                print('\nFailed to reach destination.')
                self.state = State.STATUS_OF_TASK

            elif (self.task_status == "COMPLETED" and self.home_again == True):
                self.state = State.IDLE

            elif (self.task_status == "COMPLETED"):
                Screen.nav_succ = True
                print('\nDestination Reached.')
                self.state = State.STATUS_OF_TASK

                
        elif (self.state == State.STATUS_OF_TASK):

            self.get_logger().info('State: STATUS_OF_TASK')
            print('next.goal is: ', Screen.next_goal)

            if Screen.next_goal and Screen.last_goal_in_path == False:
                self.state = State.MULTIPLE_TASKS
    

            elif self.task_status == 'COMPLETED' and Screen.program_paths == False:
                self.get_logger().info('GOAL_REACHED')

                Screen.nav_succ = None
                print('Waiting for user verification')

            elif self.task_status == 'FAILED' and Screen.program_paths == False:
                self.get_logger().info('State: GOAL_FAILED')

                Screen.nav_succ = None
                print('Goal could not be reached. Please click return to home') 

            if Screen.to_home == True:
                self.home_again = True
                print('Returning back to home')
                self.pose_name = 'home'
                self.task_to_do = self.goals['home']
                print('Task to do is: ', self.task_to_do)
                self.state = State.CREATE_TASK
            
        
        if (self.state != self.last_state):
            self.last_task_state_change_time = self.get_clock().now()
            self.last_state = self.state
            


def main(args=None):
    rclpy.init(args=args)
    robot = Robot()
    print(robot.goals)
    rclpy.spin(robot)

    #Destroy the node explicitly
    robot.destroy_node()
    rclpy.shutdown


#Multi-threading to allow FSM and GUI to run in parallel
t1 = Thread(target=main)
print(current_thread().getName())
t1.start()
robot = Robot()
print(robot.goals)
sm = ScreenManager()
MainApp().run()





