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
    
    def __init__(self, dimensions, location, parent = None):
        super(glWidget, self).__init__(parent)
        
        #get shape dimensions for the cubes we want to draw:

        self.x = int(dimensions["width"]) / 20 #scale by twenty to render 
        self.y = int(dimensions["height"]) / 20
        self.z = int(dimensions["length"]) / 20 

        #get coordinate locations for the cube we want to draw:



        #define boundaries for where we can draw the cube:

        if location["y"] is not 0.0: #we do not want to break the Y plane everything should be flat on a table
        	location["y"] == 0.0

        if location["z"] > -3.0: # any closer than -3.0 and the shape will be too close to render 
        	location["z"] == -3.0
		
        #boundaries by which cube is bounded by x coordinate
        if location["x"] > 4.0: 
        	location["x"] == 4.0
        elif location["x"] < -4.0:
        	location["x"] == -4.0
        
        self.locationX = location["x"]
        self.locationY = location["y"]
        self.locationZ =location["z"]

        print(self.locationX,self.locationY,self.locationZ)

    def paintGL(self, Width=640,Height=480):

    #define the verticies of the cube to be drawn:
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
	    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
	    glBegin(GL_LINES)
	    for edge in edges:
	    	for vertex in edge:
	    		glVertex3fv(verticies[vertex])

	    glEnd()


    def initializeGL(self, Width=640, Height=480):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, float(Width)/float(Height), 1, 50.0)
        glTranslatef(self.locationX,self.locationY,self.locationZ)
        #glTranslatef(self.locationX,self.locationY,self.locationZ)
        glRotatef(0.0,0,0,-20)
        glMatrixMode(GL_MODELVIEW)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = glWidget()
    window.show()
    sys.exit(app.exec_())
