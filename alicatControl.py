# -*- coding: utf-8 -*-
"""
Created on 26/09/25
pip install pycomm3
@author: SALLEJAUNE
"""


from PyQt6 import QtCore
from PyQt6.QtWidgets import (QApplication, QWidget, QMainWindow,
                             QVBoxLayout, QDial, QHBoxLayout, QDoubleSpinBox,
                             QLabel)
from PyQt6.QtGui import QIcon
import sys
import time
import qdarkstyle
from PyQt6.QtCore import Qt
import pathlib
import os
from alicatLib import AlicatController


class AlicatGui(QMainWindow):
    
    def __init__(self, IP='10.0.1.101', name=' Rosa (80 Bar) ', parent=None):
       
        super(AlicatGui, self).__init__(parent)
        p = pathlib.Path(__file__)
        sepa = os.sep
        
        self.icon = str(p.parent) + sepa + 'icons' + sepa
        self.setWindowIcon(QIcon(self.icon + 'LOA.png'))
        
        self.raise_()
        self.setUp()
        self.actionButton()

        self.alicat = AlicatController(IP)
        self.alicat.connect()
        device_info = self.alicat.get_device_info()
        if device_info:
            print(f"Vendor ID: {device_info['vendor_id']} ({'Alicat' if device_info['vendor_id'] == 1174 else 'Autre'})")
            print(f"Product: {device_info.get('product_name', 'N/A')}")
            print(f"Serial: {device_info['serial_number']}")
        self.setWindowTitle(name + 'Alicat s/n: ' +f"Serial: {device_info['serial_number']}")
        self.threadPressure = THREADPRESSURE(self)
        self.threadPressure.start()
        self.threadPressure.MEAS.connect(self.aff)
         
    def setUp(self):
        central = QWidget()
        main_layout = QVBoxLayout()
        central.setLayout(main_layout)
        self.setCentralWidget(central)
        # Ligne haute: Dial + affichage
        hLayout = QHBoxLayout()

        self.dial = QDial()
        self.dial.setNotchesVisible(True)
        self.dial.setWrapping(False)
        self.dial.setMinimum(0)
        self.dial.setMaximum(80)
        self.dial.setFixedSize(140, 140)
        hLayout.addWidget(self.dial)

        h1Layout = QHBoxLayout()

        self.setValueSpinBox = QDoubleSpinBox()
        self.setValueSpinBox.setDecimals(1)
        self.setValueSpinBox.setSingleStep(0.5)
        self.setValueSpinBox.setRange(0, 80)
        self.setValueSpinBox.setSuffix(' Bar')
        self.labelPressure = QLabel('Pressure Set Point')
        h1Layout.addWidget(self.labelPressure)
        h1Layout.addWidget(self.setValueSpinBox)

        h2Layout = QHBoxLayout()
        self.getValueSpinBox = QLabel('...')
        self.labelGetPressure = QLabel('Pressure')
        h2Layout.addWidget(self.labelGetPressure, alignment=Qt.AlignmentFlag.AlignCenter)
        h2Layout.addWidget(self.getValueSpinBox, alignment=Qt.AlignmentFlag.AlignCenter)
        V1Layout = QVBoxLayout()
        V1Layout.addLayout(h1Layout)
        V1Layout.addLayout(h2Layout)
        hLayout.addLayout(V1Layout)
        main_layout.addLayout(hLayout)

    def actionButton(self):
        self.dial.valueChanged.connect(self._on_dial_change)
        self.setValueSpinBox.valueChanged.connect(self._on_spin_change)
        
    def _on_dial_change(self, v: int):
        # Synchroniser le spinbox avec le dial
        self.setValueSpinBox.blockSignals(True)
        self.setValueSpinBox.setValue(float(v))
        self.setValueSpinBox.blockSignals(False)
        self.alicat.set_setpoint(float(self.setValueSpinBox.value()))

    def _on_spin_change(self, v: float):
        # Synchroniser le dial avec le spinbox
        self.dial.blockSignals(True)
        self.dial.setValue(int(v))
        self.dial.blockSignals(False)
        self.alicat.set_setpoint(float(self.setValueSpinBox.value())) 

    def aff(self, M):
        self.getValueSpinBox.setText(M + ' Bar')

    def closeEvent(self, event):
        
        self.threadPressure.stopThread()
        self.alicat.disconnect()
        time.sleep(0.5)
        event.accept()
        
        
class THREADPRESSURE(QtCore.QThread) :  
    
    
    MEAS = QtCore.pyqtSignal(str)
    
    def __init__(self, parent):
        super(THREADPRESSURE,self).__init__(parent)
        self.parent = parent
        self.stop = False
        
    def run(self):
        while self.stop is False:
            pressure = str(round(self.parent.alicat.get_pressure(),2))
            self.MEAS.emit(pressure)
            time.sleep(0.5)
            
            
            
    def stopThread(self):
         self.stop = True
         
if __name__=='__main__'  :
    appli = QApplication(sys.argv)
    appli.setStyleSheet(qdarkstyle.load_stylesheet(qt_api='pyqt6'))
    e = AlicatGui()
    
    e.show()
    appli.exec()
    