'''
Author: JoJo Meunier jmeunier@bu.edu 4/10/16

Class for PCR Task dialog windows

once parameters are set for colony task default json workspace config file will be used 
to generate the work space and display workspace in WorkSpaceTab on main GUI

'''


from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import drawCubes, get_json_data
from drawCubes import glWidget
from get_json_data import CollectData




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
        #get the data from the standard data file:
        #self.mydata = CollectData()
        #self.mydata.loadfile()
        #tip_dimensions,tip_location = self.mydata.get_tipBox_data()
        self.widget = glWidget()
        self.widget.setWindowTitle('Work Space Window')
        self.widget.show()

# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PCRTaskDialogWindow()
    window.show()
    sys.exit(app.exec_())