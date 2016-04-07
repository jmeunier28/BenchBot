"""
Open Source BenchBot GUI Application: BenchBot
Contains main function for entire application, GUI initilization and functions
Authors: 

1. Mike Ferguson www.mikeahferguson.com 3/26/2016
2. JoJo Meunier jmeunier@bu.edu 4/3/2016

License: MIT
Requires PyQt5 to be installed.
Anything Qt specific 
Uses Python 3.x

"""


import serial, time, struct
import sys, os, threading
from threading import Thread
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import DobotInverseKinematics
import serial.tools.list_ports


class NewTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(NewTaskDialogWindow,self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('CreateCustomTask.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        #connect dialog buttons

        self.diag.upload_python.clicked.connect(self.upload_python_clicked)
        self.diag.upload_ui.clicked.connect(self.upload_ui_clicked)
        self.diag.upload_json.clicked.connect(self.upload_json_clicked)
        self.diag.buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def upload_python_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open Python File', '', "Python Files (*.py)")
        print ('Path to file is:\n') 
        print (file)

    def upload_ui_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open UI File', '', "Ui Files (*.ui)")
        print ('Path to file is:\n')
        print (file)

    def upload_json_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open File', '', "JSON Files (*.json)")
        print ('Path to file is:\n')
        print (file)

    def ok_button_clicked(self):
        # code to set the new files will go here 
        print ("okay button works")

class PCRTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(PCRTaskDialogWindow,self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('TaskDialogBox.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        #connect buttons and user inputs:
        #self.diag.samples_doubleSpinBox.somethingggg
        #self.diag.primer_doubleSpinBox.somethingg
        self.diag.task_diag_buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def ok_button_clicked(self):
        #code to set standard json file for workspace of pcr
        #and use this to draw 
        pass

class ColonyTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(ColonyTaskDialogWindow, self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('CloningTaskDialogBox.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        self.diag.buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def ok_button_clicked(self):
        #code to set standard json file for workspace of pcr
        #and use this to draw 
        pass


class WorkSpaceDialog(QDialog):

    def __init__(self, parent=None):
        super(WorkSpaceDialog, self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('WorkSpaceConfig.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)


class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):

        super(DobotGUIApp, self).__init__(parent)
        Ui_MainWindow, QtBaseClass = uic.loadUiType('BenchBotMain.ui')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # connect to WorkSpace Tab and control GLWidget

        
        '''self.glWidget = GLWidget()

        self.ui.x_vert_slider.valueChanged.connect(self.glWidget.setXRotation)
        self.ui.openGLWidget.xRotationChanged.connect(self.ui.x_vert_slider.setValue)
        self.ui.y_vert_slider.valueChanged.connect(self.glWidget.setYRotation)
        self.ui.openGLWidget.yRotationChanged.connect(self.ui.y_vert_slider.setValue)
        self.ui.z_vert_slider.valueChanged.connect(self.glWidget.setZRotation)
        self.ui.openGLWidget.zRotationChanged.connect(self.ui.z_vert_slider.setValue)


        self.ui.x_vert_slider.setValue(15 * 16)
        self.ui.y_vert_slider.setValue(345 * 16)
        self.ui.z_vert_slider.setValue(0 * 16)'''

        #conenct menubar QAction File option to a QFileDialog 

        self.ui.new_file_action.triggered.connect(self.file_dialog_clicked)


        # connect to menubar QAction item options for Task bar Dialog Box

        self.new_task = NewTaskDialogWindow()
        self.task = PCRTaskDialogWindow()
        self.task2 = ColonyTaskDialogWindow()

        self.ui.new_cloning_task_action.triggered.connect(self.new_task.show) 
        self.ui.new_pcr_cloning_task_action.triggered.connect(self.task.show)
        self.ui.mike_pcr_cloning_task_action.triggered.connect(self.task.show)
        self.ui.jojo_pcr_cloning_task_action.triggered.connect(self.task.show)
        self.ui.new_colony_cloning_task_action.triggered.connect(self.task2.show)
        self.ui.mike_colony_cloning_task_action.triggered.connect(self.task2.show)
        self.ui.jojo_colony_cloning_task_action.triggered.connect(self.task2.show)

        # connect to menubar QAction item options for WorkSpaceConfig Dialog Box

        self.work = WorkSpaceDialog()
        self.ui.actionNew_Work_Space.triggered.connect(self.work.show)

        # connect serial ports list refresh button clicked event to the update serial port list function

        self.ui.pushButtonRefreshSerialPortsList.clicked.connect(self.update_serial_port_list)
        # connect move coordinates button clicked event to function to move to the coordinate specified
        self.ui.pushButtonMoveToCoordinate.clicked.connect(self.pushButtonMoveToCoordinate_clicked)
        # connect move to angles button clicked event to function to move to the angles specified
        self.ui.pushButtonMoveToAngles.clicked.connect(self.pushButtonMoveToAngles_clicked)

        # connect step buttons
        self.ui.pushButtonStepForward.clicked.connect(self.pushButtonStepForward_clicked)
        self.ui.pushButtonStepBackward.clicked.connect(self.pushButtonStepBackward_clicked)
        self.ui.pushButtonStepLeft.clicked.connect(self.pushButtonStepLeft_clicked)
        self.ui.pushButtonStepRight.clicked.connect(self.pushButtonStepRight_clicked)
        self.ui.pushButtonStepUp.clicked.connect(self.pushButtonStepUp_clicked)
        self.ui.pushButtonStepDown.clicked.connect(self.pushButtonStepDown_clicked)

        # connect serial port connection ("connect") button that's located in the configuration menu
        self.ui.pushButtonConnectToSerialPort.clicked.connect(self.pushButtonConnectToSerialPort_clicked)


        ###
        # Define application class variables.
        ###

        # create a variable for the arduino serial port object
        self.arduinoSerial = None

        # current position variables
        self.currentXPosition = 0
        self.currentYPosition = 0
        self.currentZPosition = 0



        ###
        # General initialization code
        ###

        # if there is a port available whose description starts with "Arduino", try to connect to it
        self.try_to_connect_to_an_arduino_port_on_application_start()
        # populate the serial ports list widget
        self.update_serial_port_list()

    
    def file_dialog_clicked(self):

        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open File')
        print ('Path to file is:\n'), (file) 

    def pushButtonMoveToCoordinate_clicked(self):

        # get moveTo coordinate text values from lineedits
        moveToX = self.ui.lineEditMoveToX.text()
        moveToY = self.ui.lineEditMoveToY.text()
        moveToZ = self.ui.lineEditMoveToZ.text()

        # check that the values were not empty
        if (moveToX == '' or moveToY == '' or moveToZ == ''):
            self.show_a_warning_message_box('Missing a coordinate value.',
                                            'Check that you entered a value for each dimension.',
                                            'Invalid coordinate for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToXFloat = float(moveToX)
            moveToYFloat = float(moveToY)
            moveToZFloat = float(moveToZ)
        except Exception as e:
            self.show_a_warning_message_box('Check that your coordinate values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Coordinate value conversion to float error')
            return

        self.move_to_cartesian_coordinate(moveToXFloat, moveToYFloat, moveToZFloat)


    def move_to_cartesian_coordinate(self, moveToXFloat, moveToYFloat, moveToZFloat):
         # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors (likely due to coordinates out of range being input) NEED TO ADDRESS THIS AT SOME POINT
        try:
            moveToAngles = DobotInverseKinematics.convert_cartesian_coordinate_to_arm_angles(moveToXFloat,moveToYFloat,moveToZFloat,
            DobotInverseKinematics.lengthUpperArm, DobotInverseKinematics.lengthLowerArm, DobotInverseKinematics.heightFromBase)
        except Exception as e:
            self.show_a_warning_message_box('Unknown inverse kinematics error. Check that your coordinate values are within the robot\'s range. '
                                            + 'The error is shown below:',
                                                repr(e),
                                                'Inverse Kinematics Error')
            return


        # check that inverse kinematics did not run into a range error. If it does, it should return -999 for all angles, so check that.
        if(moveToAngles[0] == -999):
            self.show_a_warning_message_box('Desired coordinate is outside of the robot\'s range.',
                                                'It is impossible for the robot arm to reach the coordinate you specified. Build longer arms if this range is desired.'
                                                + 'You will probably need higher torque stepper motors as well.',
                                                'Inverse Kinematics Range Error')
            return

        print('ik base angle')
        print(moveToAngles[0])
        print('ik upper angle')
        print(moveToAngles[1])
        print('ik lower angle')
        print(moveToAngles[2])


        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base and upper arm angles are those returned by the ik function.
        # real world lower arm angle is -1 * transformedLowerArmAngle
        if(self.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return



        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        self.arduinoSerial.write( struct.pack('f',moveToAngles[0]) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToAngles[0],3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToAngles[1],3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToAngles[2],3)))

        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        self.currentXPosition = moveToXFloat
        self.currentYPosition = moveToYFloat
        self.currentZPosition = moveToZFloat

        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )




    def pushButtonMoveToAngles_clicked(self):
        # get moveTo angle text values from lineedits
        moveToBaseAngle = self.ui.lineEditMoveToBaseAngle.text()
        moveToUpperArmAngle = self.ui.lineEditMoveToUpperArmAngle.text()
        moveToLowerArmAngle = self.ui.lineEditMoveToLowerArmAngle.text()

        # check that the values were not empty
        if (moveToBaseAngle == '' or moveToUpperArmAngle == '' or moveToLowerArmAngle == ''):
            self.show_a_warning_message_box('Missing a angle value.',
                                            'Check that you entered a value for each angle.',
                                            'Invalid angles for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToBaseAngleFloat = float(moveToBaseAngle)
            moveToUpperArmAngleFloat = float(moveToUpperArmAngle)
            moveToLowerArmAngleFloat = float(moveToLowerArmAngle)
        except Exception as e:
            self.show_a_warning_message_box('Check that your angle values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Angle value conversion to float error')
            return


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base, upper, and lower arm angles are those entered in the text box.
        if(self.check_for_angle_limits_is_valid(moveToBaseAngleFloat, moveToUpperArmAngleFloat, moveToLowerArmAngleFloat)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return


        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        transformedLowerArmAngle = moveToLowerArmAngleFloat*-1
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)

        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        self.arduinoSerial.write( struct.pack('f',moveToBaseAngleFloat) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToBaseAngleFloat,3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToUpperArmAngleFloat,3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToLowerArmAngleFloat,3)))

        """
        # need to implement forward kinematics
        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        """

        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )


    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid(self, baseAngle, upperArmAngle, lowerArmAngle):

        returnBool = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 <= upperArmAngle <= 105):
            # do nothing, return value already initialized true
            pass
        else:
            self.show_a_warning_message_box('Upper arm angle out of range.',
                                            'Upper arm must have an angle between -20 and 105 degrees. It is mechanically constrained.',
                                            'Upper Arm Range Error')
            returnBool = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.


        if (-102 <= lowerArmAngle <= 18):
            # do nothing, already initialized true
            pass
        else:
            self.show_a_warning_message_box('Lower arm angle out of range.',
                                            'Lower arm must have a real world angle between -102 and 18 degrees. It is mechanically constrained.',
                                            'Lower Arm Range Error')
            returnBool = False



        minAngleBetweenArms = ((180 - 81) + -79)
        maxAngleBetweenArms = ((180 - 51) + 21)
        angleBetweenArms = ((180 - upperArmAngle) + lowerArmAngle)

        if (minAngleBetweenArms <= angleBetweenArms <= maxAngleBetweenArms):
            # do nothing, already initialized true
            pass
        else:
            self.show_a_warning_message_box('Angle between arms out of range out of range.',
                                            'Angle between arms (the inner "elbow" angle) must be between ' +
                                            str(minAngleBetweenArms) + ' and ' + str(maxAngleBetweenArms) + '.' +
                                            ' It is mechanically constrained.',
                                            'Inner Elbow Angle Range Error')
            returnBool = False


        return returnBool


    def pushButtonStepForward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepForwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition + stepSizeFloat, self.currentYPosition, self.currentZPosition)

    def pushButtonStepBackward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepBackwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition - stepSizeFloat, self.currentYPosition, self.currentZPosition)

    def pushButtonStepLeft_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepLeftSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition - stepSizeFloat, self.currentZPosition)

    def pushButtonStepRight_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepRightSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition + stepSizeFloat, self.currentZPosition)

    def pushButtonStepUp_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepUpSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition, self.currentZPosition + stepSizeFloat)

    def pushButtonStepDown_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepDownSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition, self.currentZPosition - stepSizeFloat)


    def convert_step_size_numerical_text_to_number_plus_check_is_valid(self, stepSizeText):
        # check that the values were not empty
        if (stepSizeText == ''):
            self.show_a_warning_message_box('No step size value was entered.',
                                            'Check that you entered a value for the size of the step you tried to take.',
                                            'No step size value entered.')
            return None

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            stepSizeFloat = float(stepSizeText)
        except Exception as e:
            self.show_a_warning_message_box('Check that your step size values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Step size value conversion to float error')
            return None

        if (stepSizeFloat < 0):
            self.show_a_warning_message_box('Step sizes can only be positive.',
                                            'You entered a negative step size. Please enter a positive one. The button determines the direction.',
                                            'Invalid Step Size')
            return None

        return stepSizeFloat



    def update_serial_port_list(self):
        """
        Updates the serial ports qtablewidget in the configuration tab with qlistwidgetitems containing text values
        that are the names of the serial port. Calls function to get a list of ports. This function works for
        Windows, Mac, and Linux
        """
        # must clear the listwidget of any previous items
        self.ui.tableWidgetSerialPorts.clear()
        # get a list of all serial ports (all, not just open ones)
        listOfSerialPorts = list(serial.tools.list_ports.comports())

        # initialize table widget with number of rows, columns. add column titles
        self.ui.tableWidgetSerialPorts.setRowCount(len(listOfSerialPorts))
        self.ui.tableWidgetSerialPorts.setColumnCount(2)
        self.ui.tableWidgetSerialPorts.setHorizontalHeaderLabels(['Serial Port', 'Description'])
        # add each serial port name (string) to the list as a qlistwidgetitem with the string value
        for i,port in enumerate(listOfSerialPorts):
            self.ui.tableWidgetSerialPorts.setItem(i, 0, QTableWidgetItem(port[0]))
            self.ui.tableWidgetSerialPorts.setItem(i, 1, QTableWidgetItem(port[1]))
        # sort the list of ports by description so the "Arduino" described port is likely to show up first
        self.ui.tableWidgetSerialPorts.sortItems(1, QtCore.Qt.AscendingOrder)



    def connect_to_serial_port(self, portName, baudRate):

        connectionStatus = True
        previousArduinoSerial = self.arduinoSerial

        # try to connect to the arduino serial port
        try:
            # update the variable for arduino serial. first argument is port name
            # the second argument is the baud rate, or how fast the serial connection is. needs to be the same in firmware
            # This will reset the Arduino, and make the LED flash once.
            self.arduinoSerial = serial.Serial(portName, baudRate)
            # Must give Arduino time to reset
            #  "Any time less than this does not seem to work..." quoted from someone else on a blog
            time.sleep(1.5)
            # I think this clears the input stream?
            # I think I may not need this, but I'm including it anyways, against the advice of someone on stackexchange.
            # I don't think it does anything bad, just one more line of unnecessary code.
            self.arduinoSerial.flushInput()
        except Exception as e:
            self.show_a_warning_message_box('Unknown error connecting to the arduino serial port named: ' + portName +
                                            '. Perhaps the port is busy (being used by another application)?' +
                                            ' Code error shown below:',
                                            repr(e),
                                            'Arduino Serial Port Connection Error')
            self.arduinoSerial = previousArduinoSerial
            connectionStatus =  False

        if(connectionStatus):
            self.update_serial_port_connection_status_info(connectionStatus)
            self.ui.pushButtonConnectToSerialPort.setText('Disconnect')

        # returns whether or not connection was successful
        return connectionStatus

    # need to add some checks here and gui updates, essential for good usability
    def disconnect_from_serial_port(self):
        # this close command technically shouldn't be needed since serial object should be destroyed in the text line when set to none
        # and the serial object's destructor closes the port. Including it to be safe
        self.arduinoSerial.close()
        self.arduinoSerial = None
        self.update_serial_port_connection_status_info(False)
        self.ui.pushButtonConnectToSerialPort.setText('Connect')

    def update_serial_port_connection_status_info(self, connectionStatus):
        connected = connectionStatus #redundant, but makes code nicer to read
        # update gui connection info depending on the connection status
        if(connected):
            connectedLabelsStyleSheet = (
                    """
                        color: green
                    """
                )
            self.ui.labelSerialPortConnectionStatus.setText('Connected')
            self.ui.labelSerialPortConnectionStatus.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusPortName.setText(self.arduinoSerial.port)
            self.ui.labelSerialPortConnectionStatusPortName.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusBaudRate.setText(str(self.arduinoSerial.baudrate))
            self.ui.labelSerialPortConnectionStatusBaudRate.setStyleSheet(connectedLabelsStyleSheet)
        else:
            connectedLabelsStyleSheet = (
                    """
                        color: red
                    """
                )
            self.ui.labelSerialPortConnectionStatus.setText('Not Connected')
            self.ui.labelSerialPortConnectionStatus.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusPortName.setText('No Connection')
            self.ui.labelSerialPortConnectionStatusPortName.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusBaudRate.setText('No Connection')
            self.ui.labelSerialPortConnectionStatusBaudRate.setStyleSheet(connectedLabelsStyleSheet)



    def pushButtonConnectToSerialPort_clicked(self):


        pushButtonText = self.ui.pushButtonConnectToSerialPort.text()

        # if button is in disconnect state (meaning port already connected), execute disconnect functions
        # otherwise, execute connect functions
        if (pushButtonText == 'Disconnect'):
            self.disconnect_from_serial_port()
        else:
            # the treewidget.selectedItems() function returns a list of selected items of type QTreeWidgetItem.
            selectedSerialPorts = self.ui.tableWidgetSerialPorts.selectedItems()
            # if there is a selected port, try to connect to it
            if(len(selectedSerialPorts)):

                try:
                    baudRate = int(self.ui.lineEditBaudRate.text())
                except Exception as e:
                    self.show_a_warning_message_box('Check that your baud rate value is a number and does not contain letters.' +
                                                    'The code error is shown below:',
                                                    repr(e),
                                                    'Baud rate value conversion to int error')
                    return

                selectedSerialPortName = selectedSerialPorts[0].text()
                self.connect_to_serial_port(selectedSerialPortName, baudRate)

            else:
                self.show_a_warning_message_box('No Port Selected.','Select a port.', 'No Selected Port')


    def try_to_connect_to_an_arduino_port_on_application_start(self):

        baudRateValid = True
        connected = False
        possibleArduinoPortName = ''

        # get a list of all serial ports (all, not just open ones)
        listOfSerialPorts = list(serial.tools.list_ports.comports())

        # get the name of a potential arduino connected port based on the description
        for port in listOfSerialPorts:
            # if the description starts contains the word  'arduino', try to connect to it using a default baud rate as defined in the ui
            if ('arduino' in  port[1].lower()):
                possibleArduinoPortName = port[0]

        try:
            baudRate = int(self.ui.lineEditBaudRate.text())
        except Exception as e:
            self.show_a_warning_message_box('Check that your baud rate value is a number and does not contain letters.' +
                                        'The code error is shown below:',
                                        repr(e),
                                        'Baud rate value conversion to int error')
            baudRateValid = False

        if (baudRateValid and possibleArduinoPortName != ''):
            if (self.connect_to_serial_port(possibleArduinoPortName, baudRate)):
                connected = True



        if (connected == False):
            self.show_a_warning_message_box('Warning, the arduino is not connected to the computer.',
                                            'Try to connect to a port in the configuration settings. Ensure that the' +
                                            ' arduino is connected to the computer and not being used by another application.',
                                            'No Arduino Found on Application Start')


    def initialize_gui_upon_new_serial_port_connection(self):
        # update current position variables
        self.currentXPosition = 0
        self.currentYPosition = 0
        self.currentZPosition = 0

        # will need to update gui text

        # also note that the firmware will reset to think its at home
        # need to address this through limit switches and/or IMUs




    def show_a_warning_message_box(self, text, infoText, windowTitle):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(text)
            msg.setInformativeText(infoText)
            msg.setWindowTitle(windowTitle)
            msg.exec()

class GLWidget(QOpenGLWidget):
    xRotationChanged = pyqtSignal(int)
    yRotationChanged = pyqtSignal(int)
    zRotationChanged = pyqtSignal(int)

    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)

        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0

        self.lastPos = QPoint()

        self.trolltechGreen = QColor.fromCmykF(0.40, 0.0, 1.0, 0.0)
        self.trolltechPurple = QColor.fromCmykF(0.39, 0.39, 0.0, 0.0)

    def minimumSizeHint(self):
        return QSize(50, 50)

    def sizeHint(self):
        return QSize(400, 400)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            self.xRotationChanged.emit(angle)
            self.update()

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle
            self.yRotationChanged.emit(angle)
            self.update()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
            self.zRotationChanged.emit(angle)
            self.update()

    def initializeGL(self):
        self.gl = self.context().versionFunctions()
        self.gl.initializeOpenGLFunctions()

        self.setClearColor(self.trolltechPurple.darker())
        self.object = self.makeObject()
        self.gl.glShadeModel(self.gl.GL_FLAT)
        self.gl.glEnable(self.gl.GL_DEPTH_TEST)
        self.gl.glEnable(self.gl.GL_CULL_FACE)

    def paintGL(self):
        self.gl.glClear(
                self.gl.GL_COLOR_BUFFER_BIT | self.gl.GL_DEPTH_BUFFER_BIT)
        self.gl.glLoadIdentity()
        self.gl.glTranslated(0.0, 0.0, -10.0)
        self.gl.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        self.gl.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        self.gl.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
        self.gl.glCallList(self.object)

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        self.gl.glViewport((width - side) // 2, (height - side) // 2, side,
                side)

        self.gl.glMatrixMode(self.gl.GL_PROJECTION)
        self.gl.glLoadIdentity()
        self.gl.glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)
        self.gl.glMatrixMode(self.gl.GL_MODELVIEW)

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & Qt.LeftButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & Qt.RightButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)

        self.lastPos = event.pos()

    def makeObject(self):
        genList = self.gl.glGenLists(1)
        self.gl.glNewList(genList, self.gl.GL_COMPILE)

        self.gl.glBegin(self.gl.GL_QUADS)

        x1 = +0.06
        y1 = -0.14
        x2 = +0.14
        y2 = -0.06
        x3 = +0.08
        y3 = +0.00
        x4 = +0.30
        y4 = +0.22

        self.quad(x1, y1, x2, y2, y2, x2, y1, x1)
        self.quad(x3, y3, x4, y4, y4, x4, y3, x3)

        self.extrude(x1, y1, x2, y2)
        self.extrude(x2, y2, y2, x2)
        self.extrude(y2, x2, y1, x1)
        self.extrude(y1, x1, x1, y1)
        self.extrude(x3, y3, x4, y4)
        self.extrude(x4, y4, y4, x4)
        self.extrude(y4, x4, y3, x3)

        NumSectors = 200

        for i in range(NumSectors):
            angle1 = (i * 2 * math.pi) / NumSectors
            x5 = 0.30 * math.sin(angle1)
            y5 = 0.30 * math.cos(angle1)
            x6 = 0.20 * math.sin(angle1)
            y6 = 0.20 * math.cos(angle1)

            angle2 = ((i + 1) * 2 * math.pi) / NumSectors
            x7 = 0.20 * math.sin(angle2)
            y7 = 0.20 * math.cos(angle2)
            x8 = 0.30 * math.sin(angle2)
            y8 = 0.30 * math.cos(angle2)

            self.quad(x5, y5, x6, y6, x7, y7, x8, y8)

            self.extrude(x6, y6, x7, y7)
            self.extrude(x8, y8, x5, y5)

        self.gl.glEnd()
        self.gl.glEndList()

        return genList

    def quad(self, x1, y1, x2, y2, x3, y3, x4, y4):
        self.setColor(self.trolltechGreen)

        self.gl.glVertex3d(x1, y1, -0.05)
        self.gl.glVertex3d(x2, y2, -0.05)
        self.gl.glVertex3d(x3, y3, -0.05)
        self.gl.glVertex3d(x4, y4, -0.05)

        self.gl.glVertex3d(x4, y4, +0.05)
        self.gl.glVertex3d(x3, y3, +0.05)
        self.gl.glVertex3d(x2, y2, +0.05)
        self.gl.glVertex3d(x1, y1, +0.05)

    def extrude(self, x1, y1, x2, y2):
        self.setColor(self.trolltechGreen.darker(250 + int(100 * x1)))

        self.gl.glVertex3d(x1, y1, +0.05)
        self.gl.glVertex3d(x2, y2, +0.05)
        self.gl.glVertex3d(x2, y2, -0.05)
        self.gl.glVertex3d(x1, y1, -0.05)

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def setClearColor(self, c):
        self.gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def setColor(self, c):
        self.gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())

# main function
if __name__ == '__main__':
    # These first three lines initialize the Qt application/GUI.
    app = QApplication(sys.argv)
    window = DobotGUIApp()
    # displays the GUI
    window.show()

    # write whatever set up or logic code you want to here.


    # Says to exit the whole code when the Qt application is closed. app.exec returns some value when qt app quits
    sys.exit(app.exec_())

