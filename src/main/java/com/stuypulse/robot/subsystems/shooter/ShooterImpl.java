package com.stuypulse.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors.StatusFrame;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.FilteredRelativeEncoder;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterLowFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final CANSparkMax feederMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private final DigitalInput feederBeam;

    private final SparkPIDController leftController;
    private final SparkPIDController rightController;

    private final BStream hasNote;

    private final SmartNumber leftTargetRPM;
    private final SmartNumber rightTargetRPM;

    protected ShooterImpl() {
        leftMotor = new CANSparkMax(Ports.Shooter.LEFT_MOTOR, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Ports.Shooter.RIGHT_MOTOR, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Ports.Shooter.FEEDER_MOTOR, MotorType.kBrushless);

        leftEncoder = new FilteredRelativeEncoder(leftMotor);
        rightEncoder = new FilteredRelativeEncoder(rightMotor);

        feederBeam = new DigitalInput(Ports.Shooter.RECIEVER_IR);
        
        leftEncoder.setVelocityConversionFactor(1.2);
        rightEncoder.setVelocityConversionFactor(1.0);
        
        leftController = leftMotor.getPIDController();
        rightController = rightMotor.getPIDController();

        leftController.setP(Settings.Shooter.LEFT.PID.kP);
        leftController.setI(Settings.Shooter.LEFT.PID.kI);
        leftController.setD(Settings.Shooter.LEFT.PID.kD);
        leftController.setFF(Settings.Shooter.LEFT.FF.kV);

        rightController.setP(Settings.Shooter.RIGHT.PID.kP);
        rightController.setI(Settings.Shooter.RIGHT.PID.kI);
        rightController.setD(Settings.Shooter.RIGHT.PID.kD);
        rightController.setFF(Settings.Shooter.RIGHT.FF.kV);
        
        hasNote = BStream.create(feederBeam).not()
            .filtered(new BDebounce.Falling(Settings.Shooter.HAS_NOTE_FALLING_DEBOUNCE))
            .filtered(new BDebounce.Rising(Settings.Shooter.HAS_NOTE_RISING_DEBOUNCE));

        Motors.disableStatusFrames(leftMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(rightMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);
        Motors.disableStatusFrames(feederMotor, StatusFrame.ANALOG_SENSOR, StatusFrame.ALTERNATE_ENCODER, StatusFrame.ABS_ENCODER_VELOCITY);

        Motors.Shooter.LEFT_SHOOTER.configure(leftMotor);
        Motors.Shooter.RIGHT_SHOOTER.configure(rightMotor);
        Motors.Shooter.FEEDER_MOTOR.configure(feederMotor); 

        leftTargetRPM = new SmartNumber("Shooter/Left Target RPM", Settings.Shooter.SPEAKER.getLeftRPM());
        rightTargetRPM = new SmartNumber("Shooter/Right Target RPM", Settings.Shooter.SPEAKER.getRightRPM());
    }

    private double getLeftShooterRPM() {
        return leftEncoder.getVelocity();
    }

    private double getRightShooterRPM() {
        return rightEncoder.getVelocity();
    }

    @Override
    public boolean atTargetSpeeds() {
        return Math.abs(getLeftShooterRPM() - leftTargetRPM.get()) < Settings.Shooter.TARGET_RPM_THRESHOLD 
            && Math.abs(getRightShooterRPM() - rightTargetRPM.get()) < Settings.Shooter.TARGET_RPM_THRESHOLD;
    }

    private void setTargetSpeeds(ShooterSpeeds speeds) {
        this.leftTargetRPM.set(speeds.getLeftRPM());
        this.rightTargetRPM.set(speeds.getRightRPM());
    }

    private void setLeftShooterRPM(double rpm) {
        leftController.setReference(rpm, ControlType.kVelocity);
    }
    
    private void setRightShooterRPM(double rpm) {
        rightController.setReference(rpm, ControlType.kVelocity);
    }

    private void setFeederBasedOnState() {
        switch (getFeederState()) {
            case INTAKING:
                feederMotor.set(+Settings.Shooter.FEEDER_INTAKE_SPEED);
                break;
            case DEACQUIRING:
                feederMotor.set(-Settings.Shooter.FEEDER_DEAQUIRE_SPEED);
                break;
            case SHOOTING:
                feederMotor.set(Settings.Shooter.FEEDER_SHOOT_SPEED);
                break;
            case STOP:
                feederMotor.set(0);
                break;
            default:
                feederMotor.set(0);
                break;
        }
    }

    private void setFlywheelsBasedOnState() {
        double manualFerryDistance = Units.metersToInches(Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose()));
        switch (getFlywheelState()) {
            case SPEAKER:
                setTargetSpeeds(Settings.Shooter.SPEAKER);
                break;
            case LOW_FERRY:
                setTargetSpeeds(getLowFerrySpeeds());
                break;
            case LOW_FERRY_MANUAL:
                setTargetSpeeds(new ShooterSpeeds(ShooterLowFerryInterpolation.getRPM(manualFerryDistance)));
                break;
            case LOB_FERRY:
                setTargetSpeeds(getLobFerrySpeeds());
                break;
            case LOB_FERRY_MANUAL:
                setTargetSpeeds(new ShooterSpeeds(ShooterLobFerryInterpolation.getRPM(manualFerryDistance)));
                break;
            case STOP:
                setTargetSpeeds(new ShooterSpeeds());
                break;
            default:
                setTargetSpeeds(new ShooterSpeeds());
                break;
        }

        if (leftTargetRPM.get() == 0) {
            leftMotor.set(0);
        }
        else {
            setLeftShooterRPM(leftTargetRPM.get());
        }

        if (rightTargetRPM.get() == 0) {
            rightMotor.set(0);
        }
        else {
            setRightShooterRPM(rightTargetRPM.get());
        }
    }

    @Override
    public boolean hasNote() {
        return hasNote.get();
    }

    private ShooterSpeeds getLowFerrySpeeds() {
        Translation2d ferryZone = Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 1.5)
            : new Translation2d(0, 1.5);
        
        double distanceToFerryInInches = Units.metersToInches(SwerveDrive.getInstance().getPose().getTranslation().getDistance(ferryZone));
        
        double targetRPM = ShooterLobFerryInterpolation.getRPM(distanceToFerryInInches);
        return new ShooterSpeeds(targetRPM, 500);
    }

    private ShooterSpeeds getLobFerrySpeeds() {
        Translation2d ferryZone = Robot.isBlue()
            ? new Translation2d(0, Field.WIDTH - 1.5)
            : new Translation2d(0, 1.5);
        
        double distanceToFerryInInches = Units.metersToInches(SwerveDrive.getInstance().getPose().getTranslation().getDistance(ferryZone));

        double targetRPM = ShooterLowFerryInterpolation.getRPM(distanceToFerryInInches);
        return new ShooterSpeeds(targetRPM + 500, 500);
    }

    @Override
    public void periodic () {
        super.periodic();

        setFeederBasedOnState();
        setFlywheelsBasedOnState();

        SmartDashboard.putNumber("Shooter/Feeder Speed", feederMotor.get());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage());

        SmartDashboard.putBoolean("Shooter/Has Note", hasNote());

        SmartDashboard.putNumber("Shooter/Left RPM", getLeftShooterRPM());
        SmartDashboard.putNumber("Shooter/Right RPM", getRightShooterRPM());

        SmartDashboard.putNumber("Shooter/Left Voltage", leftMotor.getBusVoltage() * leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Right Voltage", rightMotor.getBusVoltage() * rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("Shooter/Feeder Voltage", feederMotor.getBusVoltage() * feederMotor.getAppliedOutput());

        SmartDashboard.putNumber("Shooter/Left Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Feeder Current", feederMotor.getOutputCurrent());
    }

}