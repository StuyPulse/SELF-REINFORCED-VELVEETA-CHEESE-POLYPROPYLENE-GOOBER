package com.stuypulse.robot.commands.swerve.driveAligned;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveDriveDriveAlignedManualLowFerry extends SwerveDriveDriveAligned {

    public SwerveDriveDriveAlignedManualLowFerry(Gamepad driver) {
        super(driver);
    }

    @Override
    protected Rotation2d getTargetAngle() {
        return Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle();
    }

    @Override
    protected double getDistanceToTarget() {
        return Field.getManualFerryPosition().getDistance(Field.getAmpCornerPose());
    }
}
