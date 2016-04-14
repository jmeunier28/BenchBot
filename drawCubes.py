'''

Author: JoJo Meunier jmeunier@bu.edu 4/10/16

pself.ython script using pself.yopengl to draw shapes 
this will be used to model the workspace in 3D on an openGL widget in pself.yqt

'''

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt5 import QtGui
from PyQt5.QtOpenGL import *
import sys
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import get_json_data
from get_json_data import CollectData
import sys
import json

edges = ( #in order to draw a cube we will always connect our edges in this way
		(0,1),
		(0,3),
		(0,4),
		(2,1),
		(2,3),
		(2,7),
		(6,3),
		(6,4),
		(6,7),
		(5,1),
		(5,4),
		(5,7)
		)

class glWidget(QGLWidget):
    
    def __init__(self, parent = None):
        super(glWidget, self).__init__(parent)

    def tipBox_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()
        tip_dimensions, tip_location = self.get_data.get_tipBox_data()

        self.x = int(tip_dimensions["width"]) / 20
        self.y = int(tip_dimensions["height"]) / 20
        self.z = int(tip_dimensions["length"]) / 20

        self.locationXtip = tip_location["x"]
        self.locationYtip = tip_location["y"]
        self.locationZtip = tip_location["z"]

        tip_location = [self.locationXtip,self.locationYtip,self.locationZtip]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, tip_location

    def tubeBox_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        tube_dimensions, tube_location = self.get_data.get_tubeRack_data()

        self.x = int(tube_dimensions["width"]) / 20
        self.y = int(tube_dimensions["height"]) / 20
        self.z = int(tube_dimensions["length"]) / 20

        self.locationXtube = tube_location["x"]
        self.locationYtube = tube_location["y"]
        self.locationZtube = tube_location["z"]

        tube_location = [self.locationXtube,self.locationYtube,self.locationZtube]


        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, tube_location
        
    def waste_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        waste_dimensions, waste_location = self.get_data.get_wasteContainer_data()

        self.x = int(waste_dimensions["width"]) / 20
        self.y = int(waste_dimensions["height"]) / 20
        self.z = int(waste_dimensions["length"]) / 20

        self.locationX = waste_location["x"]
        self.locationY = waste_location["y"]
        self.locationZ = waste_location["z"]

        waste_location = [self.locationX,self.locationY,self.locationZ]


        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, waste_location

    def micro_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        micro_dimensions, micro_location = self.get_data.get_microPlate_data()

        self.x = int(micro_dimensions["width"]) / 20
        self.y = int(micro_dimensions["height"]) / 20
        self.z = int(micro_dimensions["length"]) / 20

        self.locationX = micro_location["x"]
        self.locationY = micro_location["y"]
        self.locationZ = micro_location["z"]

        micro_location = [self.locationX,self.locationY,self.locationZ]


        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, micro_location



    def paintGL(self, Width=640,Height=480):

    #define the verticies of the cube to be drawn:

        tipBox_verticies, tipBox_location = self.tipBox_cube()
        tubeBox_verticies, tubeBox_location = self.tubeBox_cube()
        waste_verticies, waste_location = self.waste_cube()
        micro_verticies, micro_location = self.micro_cube()

        vert_data = [tipBox_verticies, tubeBox_verticies, waste_verticies, micro_verticies]
        local_data = [tipBox_location,tubeBox_location,waste_location,micro_location]

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        for i in range(0, len(vert_data)):
            glBegin(GL_LINES)
            for edge in edges:
                for vertex in edge:
                    glVertex3fv(vert_data[i][vertex])                    
            glEnd()
            glTranslatef(local_data[i][0],local_data[i][1],local_data[i][2])


    def initializeGL(self, Width=640, Height=480):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(Width)/float(Height), .1, 50.0)
        glRotatef(0.0,0,0,-20)
        glMatrixMode(GL_MODELVIEW)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = glWidget()
    window.show()
    sys.exit(app.exec_())