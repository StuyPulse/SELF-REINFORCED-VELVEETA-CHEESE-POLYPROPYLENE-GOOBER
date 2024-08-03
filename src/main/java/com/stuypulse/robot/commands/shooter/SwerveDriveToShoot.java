package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.constants.Settings.Alignment;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import de.erichseifert.vectorgraphics2d.intermediate.commands.Command;

public class SwerveDriveToShoot extends Command {

    private final SwerveDrive swerve;

    private final Number targetDistance;
    
    public SwerveDriveToShoot() {
        this(Alignment.PODIUM_SHOT_DISTANCE);
    }

    public SwerveDriveToShoot(Number targetDistance) {
        this(targetDistance, Alignment.DEBOUNCE_TIME);
    }

    public SwerveDriveToShoot(Number targetDistance, double debounce) {
        this.targetDistance = targetDistance;

        swerve = SwerveDrive.getInstance();

    }

}
