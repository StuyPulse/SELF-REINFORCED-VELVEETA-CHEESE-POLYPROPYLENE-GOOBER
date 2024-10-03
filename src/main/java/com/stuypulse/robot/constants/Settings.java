package com.stuypulse.robot.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartBoolean;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
  
    double DT = 1.0 / 50.0;

    boolean SAFE_MODE_ENABLED = false;

    double WIDTH = Units.inchesToMeters(36); // intake side 
    double LENGTH = Units.inchesToMeters(32);

    double CENTER_TO_FRONT_OF_INTAKE = Units.inchesToMeters(13.0);

    double HEIGHT_TO_ARM_PIVOT = Units.inchesToMeters(23.75);
    double DISTANCE_FROM_TOWER_TO_CENTER_OF_ROBOT = Units.inchesToMeters(Units.metersToInches(LENGTH) / 2 - 14.875);
    double ANGLE_BETWEEN_ARM_AND_SHOOTER = 84; // shooter is tilted up
  
    public interface Arm {
        double LENGTH = Units.inchesToMeters(16.5);

        SmartNumber MAX_VELOCITY = new SmartNumber("Arm/Max Velocity (deg/s)", SAFE_MODE_ENABLED ? 200 : 750);
        SmartNumber MAX_ACCELERATION = new SmartNumber("Arm/Max Acceleration (deg/s^2)", SAFE_MODE_ENABLED ? 200 : 700);

        SmartNumber MAX_ANGLE = new SmartNumber("Arm/Max Angle (deg)", 85);
        SmartNumber MIN_ANGLE = new SmartNumber("Arm/Min Angle (deg)", -90 + 12.25);
        
        SmartNumber MAX_ANGLE_ERROR = new SmartNumber("Arm/Max Angle Error", 2.5);

        SmartNumber AMP_ANGLE = new SmartNumber("Arm/Amp Angle", 49);
        SmartNumber LOW_FERRY_ANGLE = new SmartNumber("Arm/Low Ferry Angle", MIN_ANGLE.get());
        SmartNumber LOB_FERRY_ANGLE = new SmartNumber("Arm/Lob Ferry Angle", -50);
        SmartNumber PRE_CLIMB_ANGLE = new SmartNumber("Arm/Pre climb angle", 90);
        SmartNumber POST_CLIMB_ANGLE = new SmartNumber("Arm/Post Climb Angle", MIN_ANGLE.get());

        SmartNumber FEED_ANGLE = new SmartNumber("Arm/Feed Angle", MIN_ANGLE.get() + 0);
        SmartNumber MAX_ACCEPTABLE_FEED_ANGLE = new SmartNumber("Arm/Max Acceptable Feed Angle", FEED_ANGLE.get() + 4);

        SmartNumber SUBWOOFER_SHOT_ANGLE = new SmartNumber("Arm/Subwoofer Shot Angle", -33);

        SmartNumber BUMP_SWITCH_DEBOUNCE_TIME = new SmartNumber("Arm/Bump Switch Debounce Time", 0.02);

        double MAX_WAIT_TO_REACH_TARGET = 2.0;

        // characterize and manually tune
        public interface PID {
            SmartNumber kP = new SmartNumber("Arm/kP", 0.20000);
            SmartNumber kI = new SmartNumber("Arm/kI", 0.0);
            SmartNumber kD = new SmartNumber("Arm/kD", 0.005);
        }

        public interface Feedforward {
            SmartNumber kS = new SmartNumber("Arm/kS", 0.0); //0.44765
            SmartNumber kV = new SmartNumber("Arm/kV", 0.0); //0.10971 
            SmartNumber kA = new SmartNumber("Arm/kA", 0.0); //0.014801

            SmartNumber kG = new SmartNumber("Arm/kG", 0.14213); 
        }

        public interface Encoder {
            double GEAR_RATIO = 1.0 / (85 + 1.0/3); // 1 arm rotation (360 degrees) per 85.33 encoder ticks
        }
    }
  
    public interface Intake {
        double INTAKE_ACQUIRE_SPEED = 0.72;
        double INTAKE_DEACQUIRE_SPEED = 1.0;

        double INTAKE_FEED_SPEED = 0.4; 

        double MAX_ARM_ANGLE_FOR_INTAKE_SHOOT = Arm.MIN_ANGLE.get() + 20;
        double ARM_SPEED_THRESHOLD_TO_FEED = 1.75; // degrees per second

        double INTAKE_SHOOT_SPEED = 0.9;

        double FUNNEL_ACQUIRE = 1.0;
        double FUNNEL_DEACQUIRE = 1.0;

        double IR_DEBOUNCE = 0.0;

        double HANDOFF_TIMEOUT = 1.0;
        double MINIMUM_DEACQUIRE_TIME_WHEN_STUCK = 0.5;
    }

    public interface Shooter {
        double FEEDER_INTAKE_SPEED = 0.18;
        double FEEDER_DEAQUIRE_SPEED = 0.5;
        double FEEDER_SHOOT_SPEED = 1.0;

        double TARGET_RPM_THRESHOLD = 250;
        double MAX_WAIT_TO_REACH_TARGET = 2.0;
        
        ShooterSpeeds SPEAKER = new ShooterSpeeds(
            new SmartNumber("Shooter/Speaker RPM", 5500), 
            new SmartNumber("Shooter/Speaker RPM differential", 500)
        );

        // TODO: Find velocity
        double SPEAKER_SHOT_VELOCITY = 10.0; // m/s

        SmartNumber HAS_NOTE_FALLING_DEBOUNCE = new SmartNumber("Shooter/Has Note Falling Debounce", 0.0);
        SmartNumber HAS_NOTE_RISING_DEBOUNCE = new SmartNumber("Shooter/Has Note Rising Debounce", 0.0);

        // left runs faster than right
        public interface LEFT {
            public interface PID {
                double kP = 0.0003211;
                double kI = 0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0;
                double kV = 0.00015;
                double kA = 0;
            }
        }

        public interface RIGHT {
            public interface PID {
                double kP = 0.0002;
                double kI = 0;
                double kD = 0.0;
            }

            public interface FF {
                double kS = 0;
                double kV = 0.00019;
                double kA = 0;
            }
        }
    }
    
    public interface Swerve {
        double WIDTH = Units.inchesToMeters(36); // intake side 
        double LENGTH = Units.inchesToMeters(32); 

        double MAX_LINEAR_VELOCITY = SAFE_MODE_ENABLED ? 1.0 : 4.9;
        double MAX_LINEAR_ACCEL = SAFE_MODE_ENABLED ? 10 : 15;
        double MAX_ANGULAR_VELOCITY = SAFE_MODE_ENABLED ? 3.0 : 6.75;
        double MAX_ANGULAR_ACCEL = SAFE_MODE_ENABLED ? 25.0 : 200.0;

        String CAN_BUS_NAME = "swerve";

        SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Swerve/Align Omega Deadband", 0.05);

        // The stator current at which the wheels start to slip;
        double SLIP_CURRENT = 150.0;

        // Theoretical free speed (m/s) at 12v applied output;
        double SPEED_AT_12_VOLTS = 5.21;

        public interface Assist {
            SmartNumber ALIGN_MIN_SPEAKER_DIST = new SmartNumber("SwerveAssist/Minimum Distance to Speaker", 4); 

            double AMP_WALL_SCORE_DISTANCE = (Settings.LENGTH / 2) + Units.inchesToMeters(2.5);

            // angle PID
            SmartNumber kP = new SmartNumber("SwerveAssist/kP", 8.5);
            SmartNumber kI = new SmartNumber("SwerveAssist/kI", 0.0);
            SmartNumber kD = new SmartNumber("SwerveAssist/kD", 0.4);

            double ANGLE_DERIV_RC = 0.05;
            double REDUCED_FF_DIST = 0.75;
        }

        public interface Motion {
            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration", 4.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity", Units.degreesToRadians(540));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());

            PIDConstants XY = new PIDConstants(1.0, 0, 0.02);
            PIDConstants THETA = new PIDConstants(2.0, 0, 0.02);
        }

        public interface Encoder {
            public interface Drive {
                double WHEEL_DIAMETER = Units.inchesToMeters(4);
                double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
                double GEAR_RATIO = 1.0 / Swerve.Drive.L4;

                double POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }

            public interface Turn {
                double POSITION_CONVERSION = 1;
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60.0;
            }
        }

        public interface Turn {
            SmartNumber kP = new SmartNumber("Swerve/Turn/PID/kP", Robot.isReal() ? 9.0 : 9.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/PID/kI", Robot.isReal() ? 0.0 : 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/PID/kD", Robot.isReal() ? 0.2 : 0.0);

            SmartNumber kS = new SmartNumber("Swerve/Turn/FF/kS", Robot.isReal() ? 0.30718 : Simulation.TURN_FRICTION_VOLTAGE);
            SmartNumber kV = new SmartNumber("Swerve/Turn/FF/kV", Robot.isReal() ? 1.42659 : 0.0);
            SmartNumber kA = new SmartNumber("Swerve/Turn/FF/kA", Robot.isReal() ? 0.0036513 : 0.0);

            boolean INVERTED = true;

            double GEAR_RATIO = (150.0 / 7.0); // 21.4285714286
        }

        public interface Drive {
            SmartNumber kP = new SmartNumber("Swerve/Turn/PID/kP", Robot.isReal() ? 9.0 : 1.0);
            SmartNumber kI = new SmartNumber("Swerve/Turn/PID/kI", Robot.isReal() ? 0.0 : 0.0);
            SmartNumber kD = new SmartNumber("Swerve/Turn/PID/kD", Robot.isReal() ? 0.0 : 0.1);

            SmartNumber kS = new SmartNumber("Swerve/Drive/FF/kS", Robot.isReal() ? 0.31007 : Simulation.DRIVE_FRICTION_VOLTAGE);
            SmartNumber kV = new SmartNumber("Swerve/Drive/FF/kV", Robot.isReal() ? 1.62153 : 0.25);
            SmartNumber kA = new SmartNumber("Swerve/Drive/FF/kA", Robot.isReal() ? 0.0048373 : 0.01);

            double L2 = ((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)); // 6.74607175
            double L3 = ((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)); // 6.12244898
            double L4 = ((50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0)); // 5.35714285714
        }

        public interface FrontRight {
            boolean DRIVE_INVERTED = true;
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.1318359375);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * +0.5);
        }

        public interface FrontLeft {
            boolean DRIVE_INVERTED = false;
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.052734375);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * -0.5, WIDTH * -0.5);
        }

        public interface BackLeft {
            boolean DRIVE_INVERTED = false;
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.33154296875);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * -0.5);
        }

        public interface BackRight {
            boolean DRIVE_INVERTED = false;
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromRotations(0.192138671875 + 0.5);
            Translation2d MODULE_OFFSET = new Translation2d(LENGTH * +0.5, WIDTH * +0.5);
        }

        public interface Simulation {
            double TURN_INERTIA = 0.00001;
            double DRIVE_INERTIA = 0.00001;
            // Simulated voltage necessary to overcome friction
            double TURN_FRICTION_VOLTAGE = 0.25;
            double DRIVE_FRICTION_VOLTAGE = 0.25;
        }
    }

    public interface Alignment {
        double DEBOUNCE_TIME = 0.05;

        SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance", 0.1);
        SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance", 0.1);
        SmartNumber ANGLE_TOLERANCE = new SmartNumber("Alignment/Angle Tolerance", 6);

        SmartNumber CLIMB_SETUP_DISTANCE = new SmartNumber("Alignment/Climb/Setup Distance", Units.inchesToMeters(21.0));
        SmartNumber INTO_CHAIN_SPEED = new SmartNumber("Alignment/Climb/Into Chain Speed", 0.25);

        double MAX_ALIGNMENT_SPEED = 2.5;

        public interface Translation {
            SmartNumber kP = new SmartNumber("Alignment/Translation/kP", 6.0);
            SmartNumber kI = new SmartNumber("Alignment/Translation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Translation/kD", 0.2);
        }

        public interface Rotation {
            SmartNumber kP = new SmartNumber("Alignment/Rotation/kP", 6.0);
            SmartNumber kI = new SmartNumber("Alignment/Rotation/kI", 0.0);
            SmartNumber kD = new SmartNumber("Alignment/Rotation/kD", 0.4);

            SmartNumber ALIGN_OMEGA_DEADBAND = new SmartNumber("Alignment/Rotation/Omega Deadband", 0.05);
        }

        public interface Shoot {
            public interface Translation {
                SmartNumber kP = new SmartNumber("ShootAlign/Translation/kP", 7.5);
                SmartNumber kI = new SmartNumber("ShootAlign/Translation/kI", 0.0);
                SmartNumber kD = new SmartNumber("ShootAlign/Translation/kD", 0.7);
            }

            public interface Rotation {
                SmartNumber kP = new SmartNumber("ShootAlign/Rotation/kP", 6.0);
                SmartNumber kI = new SmartNumber("ShootAlign/Rotation/kI", 0.0);
                SmartNumber kD = new SmartNumber("ShootAlign/Rotation/kD", 0.4);
            }
        }
    }

    public interface LED {
        int LED_LENGTH = 106;
        double BLINK_TIME = .15;

        double TRANSLATION_SPREAD = 0.5;
        double ROTATION_SPREAD = 15;

        SmartBoolean LED_AUTON_TOGGLE = new SmartBoolean("LED/Auton Align Display", false);
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.MAX_LINEAR_VELOCITY);
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.MAX_LINEAR_ACCEL);
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.03);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad/s)", Swerve.MAX_ANGULAR_VELOCITY);
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad/s^2)", Swerve.MAX_ANGULAR_ACCEL);
        }
    }

    public interface Vision {
        SmartBoolean IS_ACTIVE = new SmartBoolean("Vision/Is Active", true);
        double POSE_AMBIGUITY_RATIO_THRESHOLD = 0.50;
    }

    public interface Buzz {
        double BUZZ_DURATION = 1.0;
        double BUZZ_INTENSITY = 1.0;
    }
}
