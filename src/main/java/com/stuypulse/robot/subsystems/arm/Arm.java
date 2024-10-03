package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    private static final Arm instance;

    static {
        instance = new ArmImpl();
    }

    public static Arm getInstance(){
        return instance;
    }

    public enum State {
        AMP,
        SUBWOOFER_SHOT,
        SPEAKER,
        LOW_FERRY,
        LOW_FERRY_MANUAL,
        LOB_FERRY,
        LOB_FERRY_MANUAL,
        FEED,
        STOW,
        PRE_CLIMB,
        CLIMBING,
        RESETTING
    }

    protected State state;

    protected Arm() {
        if (Robot.isReal()) {
            state = State.RESETTING;
        }
        else {
            state = State.FEED;
        }
    }

    public void setState(State state) {
        if (state != State.RESETTING) {
            this.state = state;
        }
    }

    public State getState() {
        return this.state;
    }

    public abstract boolean atTarget();

    public abstract boolean atValidFeedAngle();
    public abstract boolean atIntakeShouldShootAngle();

    public abstract double getVelocity();

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm/State", state.toString());
    }
}