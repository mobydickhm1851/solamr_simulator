#!/usr/bin/env python

import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class MyWidget(QWidget):
    def __init__(self, parent = None):
        super(MyWidget, self).__init__(parent)
        self.createLayout()
        self.spinBox.valueChanged.connect(self.slider.setValue)
        self.slider.valueChanged.connect(self.spinBox.setValue)
    
    def createLayout(self):
        self.spinBox = QSpinBox()
        self.spinBox.setPrefix("$")
        self.spinBox.setRange(0, 100)
        
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)
        
        layout = QHBoxLayout()
        layout.addWidget(self.spinBox)
        layout.addWidget(self.slider)
        self.setLayout(layout)


if __name__ == '__main__':

    try:
        app = QApplication(sys.argv)

        widget = MyWidget()
        widget.show()

        app.exec_()
    except:
        pass

