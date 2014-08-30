
#
# PS Move API - An interface for the PS Move Motion Controller
# Copyright (c) 2011 Thomas Perl <m@thp.io>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


from __future__ import division
import sys
sys.path.append("../../../psmoveapi/build")
import psmove
import time

'''
Created on Aug 19, 2014

@author: bitcraze
'''
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class CompassWidget(QWidget):

    angleChanged = pyqtSignal(float)
    
    def __init__(self, parent = None):
    
        QWidget.__init__(self, parent)
        
        self.max = -1
        self.move = psmove.PSMove()
        self.setStyleSheet("background-color:transparent;");
        self._angle = 0.0
        self._margins = 10
        self._pointText = {0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S",
                           225: "SW", 270: "W", 315: "NW"}
    
    def paintEvent(self, event):
    
        painter = QPainter()
        painter.begin(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        painter.fillRect(event.rect(), self.palette().brush(QPalette.Window))
        self.drawMarkings(painter)
        self.drawNeedle(painter)
        
        painter.end()
    
    def drawMarkings(self, painter):
    
        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        
        font = QFont(self.font())
        font.setPixelSize(10)
        metrics = QFontMetricsF(font)
        
        painter.setFont(font)
        
        i = 0
        while i < 360:
            if i == 0:
                painter.setPen(self.palette().color(QPalette.Highlight))
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52, self._pointText[i])
                painter.setPen(self.palette().color(QPalette.Shadow))
            elif i % 45 == 0:
                painter.drawLine(0, -40, 0, -50)
                painter.drawText(-metrics.width(self._pointText[i])/2.0, -52,
                                 self._pointText[i])
            else:
                painter.drawLine(0, -45, 0, -50)
            
            painter.rotate(15)
            i += 15
        
        painter.restore()
    
    def drawNeedle(self, painter):
    
        painter.save()
        painter.translate(self.width()/2, self.height()/2)
        painter.rotate(self._angle)
        scale = min((self.width() - self._margins)/120.0,
                    (self.height() - self._margins)/120.0)
        painter.scale(scale, scale)
        
        painter.setPen(QPen(self.palette().color(QPalette.Shadow), 2, Qt.SolidLine))
        painter.setBrush(self.palette().brush(QPalette.Shadow))
        
        '''
        painter.drawPolygon(
            QPolygon([QPoint(-10, 0), QPoint(0, -45), QPoint(10, 0),
                      QPoint(0, 45), QPoint(-10, 0)])
            )
        '''
        r = 8
        c = 25
        c2 = c-r/2
        painter.drawLine(0, -c2, 0, c2)
        painter.drawLine(-c2, 0, c2, 0)
        
        painter.setPen(QPen(Qt.NoPen))
        painter.drawEllipse(QPoint(0,c), r, r)
        painter.drawEllipse(QPoint(-c,0), r, r)
        painter.drawEllipse(QPoint(c,0), r, r)
        
        painter.setBrush(self.palette().brush(QPalette.Highlight))
        '''
        painter.drawPolygon(
            QPolygon([QPoint(-5, -25), QPoint(0, -45), QPoint(5, -25),
                      QPoint(0, -30), QPoint(-5, -25)])
            )
        '''
        painter.drawEllipse(QPoint(0,-c), r, r)
        
        painter.restore()
    
    def sizeHint(self):
    
        return QSize(150, 150)
    
    def angle(self):
        return self._angle
    
    @pyqtSlot(float)
    def setAngle(self, angle):
    
        if angle != self._angle:
            self._angle = angle
            self.angleChanged.emit(angle)
            self.update()
    
    angle = pyqtProperty(float, angle, setAngle)
    
    @pyqtSlot(float)
    def setValue(self, angle):
        self.setAngle(angle)
        
    def movefunc(self):
        '''
        if move.connection_type == psmove.Conn_Bluetooth:
            print('bluetooth')
        elif move.connection_type == psmove.Conn_USB:
            print('usb')
        else:
            print('unknown')
        '''
        #while True:
        #print('while')
        if self.move.poll():
            trigger_value = self.move.get_trigger()
            self.move.set_leds(trigger_value, 0, 0)
            self.move.update_leds()
            '''
            buttons = move.get_buttons()
            if buttons & psmove.Btn_TRIANGLE:
                print('triangle pressed')
                move.set_rumble(trigger_value)
            else:
                move.set_rumble(0)
            battery = move.get_battery()
            if battery == psmove.Batt_CHARGING:
                print('battery charging via USB')
            elif battery >= psmove.Batt_MIN and battery <= psmove.Batt_MAX:
                print('battery: %d / %d' % (battery, psmove.Batt_MAX))
            else:
                print('unknown battery value:', battery)
            '''
            dt = 0.001#1ms
            #print('accel:', (move.ax, move.ay, move.az))
            #print('gyro:', (move.gx, move.gy, move.gz))
            #print('magnetometer:', (move.mx, move.my, move.mz))
            #mag max = 2048
            #acc gyro max = 16016
            #div = 2048/360  #mag
            #div = 16016/360 #acc/gyro
            #gx = pitch
            #gy = -roll
            #gz = yaw
            GYROSCOPE_SENSITIVITY = 65.536
            var = self.move.gy - self.move.gz/GYROSCOPE_SENSITIVITY*dt
            #pitch
            var = -var
            print '%5d %5.3f ' % (var, var*dt)
            angle = self.angle - var*dt
            self.setAngle(angle)

if __name__ == "__main__":

    app = QApplication(sys.argv)
    
    window = QWidget() 
    compass = CompassWidget()    
    layout = QVBoxLayout()
    layout.addWidget(compass)
    window.setLayout(layout)
    
    window.show()
    
    timer = QTimer()
    timer.setSingleShot(False)
    timer.timeout.connect(compass.movefunc)
    timer.start(1)#1000hz
    
    sys.exit(app.exec_())
