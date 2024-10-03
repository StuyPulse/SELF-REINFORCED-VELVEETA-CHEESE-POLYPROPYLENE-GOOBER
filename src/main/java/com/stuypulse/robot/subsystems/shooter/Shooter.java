package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.util.ShooterSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Shooter extends SubsystemBase {
    
    private static final Shooter instance;

    static {
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance;
    }

    public enum FeederState {
        INTAKING,
        DEACQUIRING,
        SHOOTING,
        STOP
    }

    public enum FlywheelState {
        STOP,
        SPEAKER,
        LOW_FERRY,
        LOB_FERRY,
        LOW_FERRY_MANUAL,
        LOB_FERRY_MANUAL,
    }

    private FeederState feederState;
    private FlywheelState flywheelState;

    public Shooter() {
        feederState = FeederState.STOP;
        flywheelState = FlywheelState.SPEAKER;
    }

    public void setFeederState(FeederState feederState) {
        this.feederState = feederState;
    }

    public FeederState getFeederState() {
        return feederState;
    }

    public FlywheelState getFlywheelState() {
        return flywheelState;
    }

    public abstract boolean atTargetSpeeds();
    
    public abstract boolean hasNote();

    @Override
    public void periodic() {
        SmartDashboard.putString("Shooter/Feeder State", getFeederState().toString());
        
        // feeder automatically pushes note further into shooter when its sticking too far out
        if (Arm.getInstance().getState() == Arm.State.AMP && !hasNote() && feederState != FeederState.DEACQUIRING) {
            feederState = FeederState.INTAKING;
        }
        if (feederState == FeederState.INTAKING && hasNote()) {
            feederState = FeederState.STOP;
        }

        // automatic handoff
        boolean shouldHandoff = Arm.getInstance().getState() == Arm.State.FEED 
                            && Arm.getInstance().atValidFeedAngle() 
                            && !hasNote()
                            && Intake.getInstance().hasNote()
                            && Intake.getInstance().getState() != Intake.State.DEACQUIRING
                            && getFeederState() != FeederState.DEACQUIRING;

        if (shouldHandoff) {
            setFeederState(FeederState.INTAKING);
        }
        if (feederState == FeederState.INTAKING && (hasNote() || Arm.getInstance().getState() != Arm.State.FEED)) {
            setFeederState(FeederState.STOP);
        }

        // automatically determine flywheel speeds based on arm state
        switch (Arm.getInstance().getState()) {
            case LOW_FERRY:
                flywheelState = FlywheelState.LOW_FERRY;
                break;
            case LOW_FERRY_MANUAL:
                flywheelState = FlywheelState.LOW_FERRY_MANUAL;
                break;
            case LOB_FERRY:
                flywheelState = FlywheelState.LOB_FERRY;
                break;
            case LOB_FERRY_MANUAL:
                flywheelState = FlywheelState.LOB_FERRY_MANUAL;
                break;
            case SPEAKER:
                flywheelState = FlywheelState.SPEAKER;
                break;
            case CLIMBING:
                flywheelState = FlywheelState.STOP;
                break;
            case PRE_CLIMB:
                flywheelState = FlywheelState.STOP;
                break;
            default:
                break;
        }
    }
}
