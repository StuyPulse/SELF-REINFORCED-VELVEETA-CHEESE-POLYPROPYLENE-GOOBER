package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports.Swerve;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.FeederState;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public abstract class Intake extends SubsystemBase {

    private static final Intake instance;

    static {
        instance = new IntakeImpl();
    }

    public static Intake getInstance() {
        return instance;
    }

    public enum State {
        DEACQUIRING,
        ACQUIRING,
        FEEDING,
        SHOOTING,
        STOP
    }

    private State state;

    protected Intake() {
        this.state = State.STOP;
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return this.state;
    }

    public abstract boolean hasNote();

    @Override
    public void periodic() {
        if (state == State.ACQUIRING && hasNote()) {
            state = State.STOP;
        }

        // automatic handoff
        boolean shouldHandoff = Arm.getInstance().getState() == Arm.State.FEED 
                            && Arm.getInstance().atValidFeedAngle() 
                            && !Shooter.getInstance().hasNote()
                            && hasNote()
                            && getState() != Intake.State.DEACQUIRING
                            && Shooter.getInstance().getFeederState() != Shooter.FeederState.DEACQUIRING;
        
        if (shouldHandoff) {
            setState(State.FEEDING);
        }
        if (state == State.FEEDING && !shouldHandoff) {
            setState(State.STOP);
        }

        // run the intake when the arm is moving up from a low angle (to prevent intake from gripping it)
        // run the intake when shooting in case the intake is holding onto the note also
        boolean shouldShoot = (Shooter.getInstance().getFeederState() == Shooter.FeederState.SHOOTING && Arm.getInstance().atIntakeShouldShootAngle())
                            || (Arm.getInstance().atIntakeShouldShootAngle() && Arm.getInstance().getVelocity() > Settings.Intake.ARM_SPEED_THRESHOLD_TO_FEED);

        if (state == State.STOP && shouldShoot) {
            setState(State.SHOOTING);
        }
        if (state == State.SHOOTING && !shouldShoot) {
            setState(State.STOP);
        }

        // SmartDashboard.putString("Intake/State", state.name());
    }
}