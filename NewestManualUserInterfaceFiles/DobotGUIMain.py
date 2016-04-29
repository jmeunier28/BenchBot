"""
Open Source Dobot GUI Application: open-dobot-gui
Contains main function for entire application, GUI initilization and functions
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT

Requires PyQt5 to be installed.

Anything Qt specific (functions, classes, etc.. starts with the letter Q)

The GUI template is created by a program called QtDesigner, which spits out a .ui file, which is basically just an XML
file that describes the GUI elements. In the designer, the elements are given object names. The first step of the code
below is to load the .ui file and get a reference to it.
"""




import serial, time, struct, math
import sys, os, threading
from threading import Thread
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem
from PyQt5 import uic, QtCore, QtGui
import DobotInverseKinematics
import serial.tools.list_ports

# This loads the GUI from the .ui file that is created by QtDesigner. The .ui file should be in the same folder as this
# python file (or specify different path).
Ui_MainWindow, QtBaseClass = uic.loadUiType('DobotMainUi.ui')


# Here, a class is defined to represent the entire GUI. It is derived from a Qt class named QMainWindow, which
# corresponds to the GUI type specified in QtDesigner. All of the functional aspects (as opposed to design aspects) of
# the GUI are defined in this class. For example, what happens when a user presses a button.
class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):
        # I'm a python noob, but I'm guessing this means initialize the parent class. I imagine all the super classes
        # have to be explicitly initialized.
        super(DobotGUIApp, self).__init__(parent)
        # This sets up the ui variable of this class to refer to the loaded .ui file.
        self.ui = Ui_MainWindow()
        # This call is required. Does whatever set up is required, probably gets references to the elements and so on.
        self.ui.setupUi(self)

        # Anything named after self.ui. (e.g. self.ui.x) means you are referring to an object name x that corresponds
        # to an element in the gui. Qt uses a signals and slots framework to define what happens in response to UI
        # events. You'll have to look that up if you're interested in it.


        ###
        # Connect gui elements in the .ui file to event handling functions.
        ###

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
        startXYZ = DobotInverseKinematics.get_cartesian_coordinate_from_angles_using_forward_kinematics(0,90,0)
        self.currentXPosition = startXYZ[0]
        self.currentYPosition = startXYZ[1]
        self.currentZPosition = startXYZ[2]
        print(startXYZ)

        self.stepperPositionsOnALine = []
        self.stepperSequenceToDrawALine = []
        #START STEPPER MOTOR SETTINGS
        #The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
        stepperMotorStepsPerRevolution = 200
        #I'm using a ramps 1.4 board with all 3 jumpers connected, which gives me a microstepping mode of 1/16.
        #In other words, the motor is set up so it takes 16 steps to move 1 of the default steps.
        #microstepping jumper guide for the a4988 stepper driver: https://www.pololu.com/product/1182
        baseMicrosteppingMultiplier = 16
        upperArmMicrosteppingMultiplier = 16
        lowerArmMicrosteppingMultiplier = 16
        """
        //The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders.
        //It basically just means that the stepper motor is rotating a smaller gear. That smaller gear is in turn rotating a larger one.
        //The gears are set up such that rotating the smaller gear by some number of degrees rotates the larger one by a tenth of that number of degrees (10:1 ratio)
        //The bigger gears are actually moving the arm, so the number of steps is increased by a factor of 10 (the gear ratio).
        """
        stepperPlanetaryGearBoxMultiplier = 10
        #This variable will hold the aqctual number of steps per revolution and is calculate by multiplying the three previous variables together.
        #calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
        self.baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        self.upperArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * upperArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        self.lowerArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * lowerArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        #END STEPPER MOTOR SETTINGS
        self.basePos = 0
        self.upperPos = 0
        self.lowerPos = 0

        ###
        # General initialization code
        ###

        # if there is a port available whose description starts with "Arduino", try to connect to it
        self.try_to_connect_to_an_arduino_port_on_application_start()
        # populate the serial ports list widget
        self.update_serial_port_list()









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

        # divide a line between starting and end points into many equally spaced points and move to each of those points,
        # so the arm moves in a straight line
        startingPointX = self.currentXPosition
        startingPointY = self.currentYPosition
        startingPointZ = self.currentZPosition
        directionX = moveToXFloat - startingPointX
        directionY = moveToYFloat - startingPointY
        directionZ = moveToZFloat - startingPointZ
        linearMovementResolution = 100

        #clear the stepper sequence for the line and the stepper positions on the line
        self.stepperPositionsOnALine[:] = []
        self.stepperSequenceToDrawALine[:] = []

        for i in range(1, linearMovementResolution+1):
            nextPointX = startingPointX + (directionX * (i/linearMovementResolution))
            nextPointY = startingPointY + (directionY * (i/linearMovementResolution))
            nextPointZ = startingPointZ + (directionZ * (i/linearMovementResolution))
            self.move_to_cartesian_coordinate(nextPointX, nextPointY, nextPointZ)
            """
            print('i:')
            print(i)
            print(nextPointX)
            print(nextPointY)
            print(nextPointZ)
            """

        self.generate_line_step_sequence()


        debug = False#note that you need to uncomment testlist.append[] and print(arduinoBuffer) if you want to debug (each occurs twice)




        #note, this is NOT the MAX_INPUT setting on the arduino. It should be 1/3 of that value
        maxStepDataTuplesCanSend = 1000
        maxStepDataCommands = maxStepDataTuplesCanSend * 3
        numLineHeaderData = 5
        numLineFooterData = 1
        numHeaderFooterData = numLineHeaderData + numLineFooterData
        #number of dataSets
        numStepDataTuples = (len(self.stepperSequenceToDrawALine)-numHeaderFooterData)/3
        numStepDataCommands = numStepDataTuples*3
        numDataSets = math.ceil(numStepDataTuples/maxStepDataTuplesCanSend)


        #send header data to arduino
        for h in range(0,numLineHeaderData):
            self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[h]) )



        if(debug):
            print(self.stepperSequenceToDrawALine)
            print('numStepDataTuples: ' + str(numStepDataTuples))
            print('numDataSets BEFORE rounding: ' + str(numStepDataTuples/maxStepDataTuplesCanSend))
            print('numDataSets: ' + str(numDataSets))

            print('start counting')

        i=0
        arduinoReady = False
        arduinoBuffer = ''
        testlist = []
        #iterate through guaranteed full sets
        for i in range(0,numDataSets-1):
            dataPacketStartPos = int(numLineHeaderData + (i * maxStepDataCommands))
            dataPacketEndPos = int(maxStepDataCommands) + dataPacketStartPos
            for j in range(dataPacketStartPos,dataPacketEndPos):
                self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[j]) )
                #print(j)
                testlist.append(j)
            if(debug): #note that you need to uncomment testlist.append[]
                print(testlist)
                print(len(testlist))
                testlist = []
            while(arduinoReady == False):
                if(self.arduinoSerial.in_waiting > 1):
                    arduinoBuffer = self.arduinoSerial.read(self.arduinoSerial.in_waiting).decode().strip()
                    if('d' in arduinoBuffer):
                        #print(arduinoBuffer)
                        arduinoReady = True
            arduinoReady = False


        # i is 1 less than the end value of the range when exiting the forloop. I want to be the same value
        # if i is zero, than means it didn't enter the for loop, so don't change it
        if(i > 0):
            i+=1


        #iterate through last set, may not be full
        dataPacketStartPos = int(numLineHeaderData + (i * maxStepDataCommands))
        #dataPacketEndPos = int(numStepDataCommands + numLineHeaderData)
        dataPacketEndPos = len(self.stepperSequenceToDrawALine)
        for j in range(dataPacketStartPos,dataPacketEndPos):
            self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[j]) )
            #print(j)
            #print(self.stepperSequenceToDrawALine[j])
            testlist.append(j)
        if(debug): #note that you need to uncomment testlist.append[]
            print(testlist)
            print(len(testlist))
        while(arduinoReady == False):
                time.sleep(.1)
                arduinoBuffer = self.arduinoSerial.read(self.arduinoSerial.in_waiting ).decode().strip()
                if('d' in arduinoBuffer):
                    print(arduinoBuffer)
                    arduinoReady = True
        arduinoReady = False

        #numStepsInLastDataSet = len(self.stepperSequenceToDrawALine) - ((numDataSets-1)*maxStepsCanSend)




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




        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90


        debug = False
        if(debug):
            print('ik base angle')
            print(moveToAngles[0])
            print('ik upper angle')
            print(moveToAngles[1])
            print('ik lower angle')
            print(moveToAngles[2])
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


        self.convert_angles_to_stepper_positions(moveToAngles[0],transformedUpperArmAngle,transformedLowerArmAngle)


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

        """
        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,16 ):
            print ( self.arduinoSerial.readline() )
        """


    def convert_angles_to_stepper_positions(self, baseAngle,upperArmAngle,lowerArmAngle):

        debug = False

        print(baseAngle)
        baseStepNumber = int(( (abs(baseAngle)/360) * self.baseActualStepsPerRevolution ) + 0.5)
        print(baseStepNumber)
        #need this because of the abs value function, which is needed for proper rounding
        if (baseAngle < 0):
            baseStepNumber *= -1



        upperArmStepNumber = int(( (abs(upperArmAngle)/360) * self.upperArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (upperArmAngle < 0):
            upperArmStepNumber *= -1


        lowerArmStepNumber = int(( (abs(lowerArmAngle)/360) * self.lowerArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (lowerArmAngle < 0):
            lowerArmStepNumber *= -1

        #necessary to reverse the direction in which the steppers move, so angles match my defined angles

        baseStepNumber *= -1
        upperArmStepNumber *= -1
        lowerArmStepNumber *= 1

        self.stepperPositionsOnALine.append(baseStepNumber)
        self.stepperPositionsOnALine.append(upperArmStepNumber)
        self.stepperPositionsOnALine.append(lowerArmStepNumber)


        if(debug):
            print("Base Angle")
            print(baseAngle)
            print("Base Step Number")
            print(baseStepNumber)
            print("Upper Arm Angle")
            print(upperArmAngle)
            print("Upper Arm Step Number")
            print(upperArmStepNumber)
            print("Lower Arm Angle")
            print(lowerArmAngle)
            print("Lower Arm Step Number")
            print(lowerArmStepNumber)



    def generate_line_step_sequence(self):

        self.stepperSequenceToDrawALine.append('l')
        print(self.stepperPositionsOnALine)
        #is it true that moving along a line means that the steppers will always be moving in the same direction? I'm assuming it does, but I need to verify this.
        #set the direction of the steppers. only need to check the first position to move to, to get the direction, since moving in a straight line
        if((self.stepperPositionsOnALine[0] - self.basePos) > 0):
            #digitalWrite(E_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(E_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')
        if((self.stepperPositionsOnALine[1] - self.upperPos) > 0):
            #digitalWrite(X_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(X_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')
        if((self.stepperPositionsOnALine[2] - self.lowerPos) > 0):
            #digitalWrite(Y_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(Y_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')


        for j in range(0,len(self.stepperPositionsOnALine),3):
            self.generate_step_sequence_from_position_to_position(abs(self.stepperPositionsOnALine[j] - self.basePos),
                                                                  abs(self.stepperPositionsOnALine[j+1] - self.upperPos),
                                                                  abs(self.stepperPositionsOnALine[j+2] - self.lowerPos))
            self.basePos = self.stepperPositionsOnALine[j]
            self.upperPos = self.stepperPositionsOnALine[j+1]
            self.lowerPos = self.stepperPositionsOnALine[j+2]

        self.stepperSequenceToDrawALine.insert(4,'s')
        self.stepperSequenceToDrawALine.append('e')
        #self.stepperSequenceToDrawALine.insert(4,len(self.stepperSequenceToDrawALine)-4)


        """
        for i in range(0,len(self.stepperSequenceToDrawALine),3):
            a = self.stepperSequenceToDrawALine[i]
            b = self.stepperSequenceToDrawALine[i+1]
            c = self.stepperSequenceToDrawALine[i+2]
            print('Stepper Sequence')
            print(str(a) +','+ str(b) +','+ str(c))
        """
        print('Stepper Sequence')
        print(self.stepperSequenceToDrawALine)


    def generate_step_sequence_from_position_to_position(self,numBaseSteps,numUpperArmSteps,numLowerArmSteps):


        #of the 3 stepper motors determine which one requires the most steps
        max_steps = float(max(max(numBaseSteps, numUpperArmSteps), numLowerArmSteps))#needs to be float so the rounding in the divisions work

        if(max_steps == 0):
            return

        baseStepSpace = max_steps+1
        upperStepSpace = max_steps+1
        lowerStepSpace = max_steps+1

        baseRemainder = 0
        upperRemainder = 0
        lowerRemainder = 0


        if(numBaseSteps != 0):
          baseStepSpace = int((max_steps / numBaseSteps) +.5)#round to nearest int
          baseRemainder = int(max_steps/baseStepSpace)#round to lowest int
          baseRemainder = numBaseSteps - baseRemainder#finish the calculation

        if(numUpperArmSteps != 0):
          upperStepSpace = int((max_steps / numUpperArmSteps) + 0.5)#round to nearest int
          upperRemainder = int(max_steps/upperStepSpace)#round to lowest int
          upperRemainder = numUpperArmSteps - upperRemainder#finish the calculation

        if(numLowerArmSteps != 0):
          lowerStepSpace = int((max_steps / numLowerArmSteps) + .5)#round to nearest int
          lowerRemainder = int(max_steps/lowerStepSpace)#round to lowest int
          lowerRemainder = numLowerArmSteps - lowerRemainder#finish the calculation

        """
        print(baseStepSpace)
        print(upperStepSpace)
        print(lowerStepSpace)
        print(baseRemainder)
        print(upperRemainder)
        print(lowerRemainder)
        """

        baseStepSpacei = 0
        upperStepSpacei = 0
        lowerStepSpacei = 0

        baseTaken = 0
        upperTaken = 0
        lowerTaken = 0

        baseflag = False
        upperflag = False
        lowerflag = False

        #step the motors at the same time by moving them 1 step at essentially the same time
        #Must alternate between HIGH and LOW signals to step the motors. I don't know the physics of why though. Just look up one of the million tutorials on stepper motors if you're curious why.

        for i in range(0,int(max_steps)):
            if ((baseStepSpacei == 0) and (baseTaken < numBaseSteps)):
                #digitalWrite(baseStepPin, HIGH);//only step the motor if it has more steps remaining to take
                baseflag = True
            if ((upperStepSpacei == 0) and (upperTaken < numUpperArmSteps)):
                #digitalWrite(upperArmStepPin, HIGH);
                upperflag = True
            if ((lowerStepSpacei == 0) and (lowerTaken < numLowerArmSteps)):
                #digitalWrite(lowerArmStepPin, HIGH);
                lowerflag = True

                #delay(1);

            if (baseflag):
                #digitalWrite(baseStepPin, LOW);
                baseStepSpacei = baseStepSpace
                baseTaken += 1
                baseflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            if (upperflag):
                #digitalWrite(upperArmStepPin, LOW);
                upperStepSpacei = upperStepSpace
                upperTaken += 1
                upperflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            if (lowerflag):
                #digitalWrite(lowerArmStepPin, LOW);
                lowerStepSpacei = lowerStepSpace
                lowerTaken += 1
                lowerflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            baseStepSpacei -= 1
            upperStepSpacei -= 1
            lowerStepSpacei -= 1

        #NEED TO ADDRESS THIS!!!!!!
        """
        #take any remaining steps
        for i in range(0,baseRemainder):
            #digitalWrite(baseStepPin, HIGH);
            #delay(1);
            #digitalWrite(baseStepPin, LOW);
            baseTaken += 1


        for i in range(0,upperRemainder):
            #digitalWrite(upperArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(upperArmStepPin, LOW);
            upperTaken += 1

        for i in range(0,lowerRemainder):
            #digitalWrite(lowerArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(lowerArmStepPin, LOW);
            lowerTaken += 1


        print(baseTaken)
        print(upperTaken)
        print(lowerTaken)
        """

        """
        a,b,c = -1

        for i in range(0,len(self.stepperSequenceToDrawALine),3):
            a = self.stepperSequenceToDrawALine[i]
            b = self.stepperSequenceToDrawALine[i+1]
            c = self.stepperSequenceToDrawALine[i+2]
            print(a)
            print(b)
            print(c)
        """










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

        """
        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )
        """


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




