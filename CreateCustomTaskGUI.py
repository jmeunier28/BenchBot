'''

Author: Jojo Meunier jmeunier@bu.edu 4/10/16

Class that contains Create New Task Dialog window 
Allows user to upload python, ui and json files 

uploading the python and ui files will give the user freedom and flexibility to 
define a new task with unique parameters 

uploading a custom json file will generate a new workspace

'''



from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import json


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
        file2 = file_dialog.getSaveFileName(self,'Open JSON','',"JSON Files (*.json)")
        print ('Path to file is:\n')
        print (file)
        print(file2)

    def ok_button_clicked(self):
        # code to set the new files will go here 
        print ("okay button works")

# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = NewTaskDialogWindow()
    window.show()
    sys.exit(app.exec_())

