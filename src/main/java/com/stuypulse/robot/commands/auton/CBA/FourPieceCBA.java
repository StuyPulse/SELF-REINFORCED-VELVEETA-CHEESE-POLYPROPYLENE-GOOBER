package com.stuypulse.robot.commands.auton.CBA;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.UntilNoteShot;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.constants.Settings.Auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceCBA {
    
    public FourPieceCBA(PathPlannerPath... path) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(Auton.SHOOTER_STARTUP_DELAY)
                    .andThen(new ShooterScoreSpeaker()),
                
                SwerveDriveToPose.speakerRelative(-15)
                    .withTolerance(0.03, 0.03, 3)
            ),

            new UntilNoteShot(0.7),

            new FollowPathAndIntake(path[0]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 3),
            new UntilNoteShot(),

            new FollowPathAndIntake(path[1]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 3),
            new UntilNoteShot(),

            new FollowPathAndIntake(path[2]),
            new SwerveDriveToShoot()
                .withTolerance(0.03, 3),
            new UntilNoteShot(0.7)
        );
    }

}
