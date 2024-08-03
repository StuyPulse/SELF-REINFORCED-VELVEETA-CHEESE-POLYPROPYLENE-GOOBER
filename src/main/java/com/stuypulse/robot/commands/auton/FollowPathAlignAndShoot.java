package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathAlignAndShoot extends SequentialCommandGroup {
    
    public double getPathTime(PathPlannerPath path) {
        return path.getTrajectory(new ChassisSpeeds(), path.getStartingDifferentialPose().getRotation())
            .getTotalTimeSeconds();
    }
    
    public FollowPathAlignAndShoot(PathPlannerPath path, Command alignCommand) {
        this(path, alignCommand, false);
    }

    public FollowPathAlignAndShoot(PathPlannerPath path, Command alignCommand, boolean noteShot) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(path),
                new WaitCommand(getPathTime(path) - Auton.SHOOTER_START_PRE)
                    .andThen(new ShooterScoreSpeaker())
            ),
            alignCommand,
            new ShooterWaitForTarget()
                .withTimeout(0.5)
        );

        if (noteShot)
            addCommands(new UntilNoteShot(0.75));
        else
            addCommands(new ShooterScoreSpeaker());
    }

}