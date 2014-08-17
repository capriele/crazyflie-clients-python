#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Used for sending control setpoints to the Crazyflie
"""

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
import struct
import math
#from cfclient.utils.input import JoystickReader #used for hold-mode

class Commander():
    """
    Used for sending control setpoints to the Crazyflie
    """

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie

        #########################
        ##        MODES        ##
        #########################
        self._x_mode = False
        self._carefree_mode = True
        self._position_mode = False
        self._hold_mode = False

        #########################
        ##        UTILS        ##
        #########################
        self._yaw = 0            #used to convert copter's current yaw to radians
        self._actualPoint = None #copter's current position update every 100ms
        self._oldThrust = 0
        self._delta = 5          #used to stabilize the copter

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def set_client_carefreemode(self, enabled):
        """
        Enable/disable the client side CareFree-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter direction is indipendent from its current yaw.
        """
        self._carefree_mode = enabled

    def set_client_positionmode(self, enabled):
        """
        Enable/disable the client side Position-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter manteins current position.
        N.W.: the user can only control the throttle!!!
        """
        self._position_mode = enabled

    def set_client_holdmode(self, enabled):
        """
        Enable/disable the client side Position-mode. 
        When enabled this recalculates the setpoints before sending them to the Crazyflie
        so that the copter manteins current position.
        N.W.: the user can control the copter normally but when he release the button the copter 
              will mantain its current position 
        """
        self._hold_mode = enabled

    def setActualPoint(self, data):
        self._actualPoint = data

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        if roll != 0 or pitch != 0 or yaw != 0 or thrust != 0:
            self._oldThrust = 0

        #if self._actualPoint is not None:
            #self._oldThrust = self._actualPoint['stabilizer.thrust']

        #if self._x_mode:
            #roll = 0.707 * (roll - pitch)
            #pitch = 0.707 * (roll + pitch)
        #elif self._carefree_mode and self._actualPoint is not None:#elif = else if
            # roll = x, pitch = y, yaw = A, A >= 0
            # x' = x*cos(A) - y*sin(A)
            # y' = x*sin(A) + y*cos(A)
            # A < 0
            # x' =  x*cos(A) + y*sin(A)
            # y' = -x*sin(A) + y*cos(A)
            ##currentYaw = self._actualPoint["stabilizer.yaw"]
            ##self._yaw = math.radians(currentYaw)
            ##cosy = math.cos(self._yaw)
            ##siny = math.sin(self._yaw)

            #print "Roll: %3.3f -- Pitch: %3.3f -- Yaw: %3.3f" % (self._actualPoint["stabilizer.roll"], self._actualPoint["stabilizer.pitch"], currentYaw)
            #print "Degree Yaw: %3.3f -- Radians Yaw: %3.3f" % (currentYaw, self._yaw)

            ##roll1 = roll
            #if self._yaw >= 0:
            #    roll  = roll*cosy - pitch*siny
            #    pitch = roll1*siny + pitch*cosy
            #else:
            ##roll  = roll*cosy + pitch*siny
            ##pitch = pitch*cosy - roll1*siny

        #elif self._position_mode and self._actualPoint is not None:
            #roll = 0
            #pitch = 0
            #yaw = self._actualPoint['stabilizer.yaw']
        
        #if self._hold_mode and self._actualPoint is not None: #e non premo nessun tasto sul jaystick
        #if self._hold_mode and self._oldThrust != 0: 
        ##########################
        ## STABILIZE THE COPTER ##
        ##########################
        if roll == 0 and pitch == 0 and self._actualPoint is not None:
            accX = self._actualPoint['acc.x']
            accY = self._actualPoint['acc.y']
            accZ = self._actualPoint['acc.z']
            rollAcc = math.atan2(accX, accZ)*180/math.pi
            pitchAcc = math.atan2(accY, accZ)*180/math.pi
            roll  -= self._actualPoint['stabilizer.roll'] + rollAcc
            pitch -= self._actualPoint['stabilizer.pitch'] + pitchAcc
        #if self._actualPoint is not None:
            #if (roll == 0 and math.fabs(self._actualPoint['stabilizer.roll']) <= self._delta) or (pitch == 0 and math.fabs(self._actualPoint['stabilizer.pitch']) <= self._delta):
            #    result = self.ComplementaryFilter(roll, pitch)
            #    roll = result[0]
            #    pitch = result[1]
            #if roll == 0 and math.fabs(self._actualPoint['stabilizer.roll']) <= self._delta:
                #roll = math.atan2(self._actualPoint['acc.x'], self._actualPoint['acc.z'])
                #-self._actualPoint['stabilizer.roll']/2
            #if pitch == 0 and math.fabs(self._actualPoint['stabilizer.pitch']) <= self._delta:
                #pitch = math.atan2(self._actualPoint['acc.y'], self._actualPoint['acc.z'])
                #-self._actualPoint['stabilizer.pitch']/2
            #if yaw == 0 and math.fabs(self._actualPoint['stabilizer.yaw']) <= self._delta:
            #    yaw = -self._actualPoint['stabilizer.yaw']/2
                
        #print "Roll: %3.3f -- Pitch: %3.3f -- Yaw: %3.3f" % (roll, pitch, yaw)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        pk.data = struct.pack('<fffH', roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)
        

    def ComplementaryFilter(self, roll, pitch):
        #ACCELEROMETER_SENSITIVITY = 8192.0
        GYROSCOPE_SENSITIVITY = 65.536
        dT = 1/500 # 500Hz 2 ms sample rate!
        a = 0.93
        b = 1-a
        # Integrate the gyroscope data -> int(angularSpeed) = angle
        roll  -= (self._actualPoint['stabilizer.pitch'] / GYROSCOPE_SENSITIVITY) * dT; # Angle around the X-axis
        pitch += (self._actualPoint['stabilizer.roll'] / GYROSCOPE_SENSITIVITY) * dT; # Angle around the Y-axis
    
        # Compensate for drift with accelerometer data if !bullshit
        # Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
        forceMagnitudeApprox = math.fabs(self._actualPoint['acc.x']) + math.fabs(self._actualPoint['acc.y']) + math.fabs(self._actualPoint['acc.z']);
        if (forceMagnitudeApprox > 8192 and forceMagnitudeApprox < 32768):
            # Turning around the Y axis results in a vector on the X-axis
            rollAcc = math.atan2(self._actualPoint['acc.x'], self._actualPoint['acc.z']) * 180 / math.pi
            roll = roll * a + rollAcc * b;
    
            # Turning around the X axis results in a vector on the Y-axis
            pitchAcc = math.atan2(self._actualPoint['acc.y'], self._actualPoint['acc.z']) * 180 / math.pi
            pitch = pitch * a + pitchAcc * b;
        result = []
        result.append(roll)
        result.append(pitch)
        return result

