/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }
    
    public interface Gyro {
        int PIGEON2 = 5;
    }

    public interface Swerve {
        public interface BackRight {
            int DRIVE = 10;
            int TURN  = 11;
            int ENCODER = 1;
        }

        public interface FrontRight {
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 2;
        }

        public interface FrontLeft {
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 3;
        }

        public interface BackLeft {
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 4;
        }
    }
}
