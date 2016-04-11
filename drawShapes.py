'''

Author: JoJo Meunier jmeunier@bu.edu 4/10/16

python script using pyopengl to draw shapes 
this will be used to model the workspace in 3D on an openGL widget in pyqt

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

verticies = (
		(1,-1,-1),
		(1,1,-1),
		(-1,1,-1),
		(-1,-1,-1),
		(1,-1,1),
		(1,1,1),
		(-1,-1,1),
		(-1,1,1)
		)
edges = (
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

    def paintGL(self):

    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    	glBegin(GL_LINES)
    	for edge in edges:
    		for vertex in edge:
    			glVertex3fv(verticies[vertex])
    	glEnd()

    '''def resizeGL(self, w, h):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-50, 50, -50, 50, -50.0, 50.0)
        glViewport(0, 0, w, h)'''

    def initializeGL(self, Width=640, Height=480):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(80.0, float(Width)/float(Height), 0.1, 50.0)
        glTranslatef(0.0,0.0,-3.0)
        glRotatef(45.0,30.0,0,-20)
        glMatrixMode(GL_MODELVIEW)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = glWidget()
    window.show()
    sys.exit(app.exec_())
