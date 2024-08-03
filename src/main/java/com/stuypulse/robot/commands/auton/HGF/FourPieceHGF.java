package com.stuypulse.robot.commands.auton.HGF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.auton.FollowPathAlignAndShoot;
import com.stuypulse.robot.commands.auton.FollowPathAndIntake;
import com.stuypulse.robot.commands.auton.UntilNoteShot;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FourPieceHGF {
    
     public FourPieceHGF(PathPlannerPath... paths) {
        addCommands(
            new ParallelCommandGroup(
                new WaitCommand(0.25)
                    .andThen(new ShooterScoreSpeaker()),

                SwerveDriveToPose.speakerRelative(-55)
                    .withTolerance(0.1, 0.1, 2) 
            ),

            new ShooterWaitForTarget(),
            new UntilNoteShot(0.75),

            new FollowPathAndIntake(paths[0]),
            new FollowPathAlignAndShoot(paths[1], new FastAlignShootSpeakerRelative(-45, 1.0)),
            new FollowPathAlignAndShoot(paths[1], SwerveDriveToPose.speakerRelative(-45)),
            new FollowPathAndIntake(paths[2]),
            new FollowPathAlignAndShoot(paths[3], new FastAlignShootSpeakerRelative(-45)),
            new FollowPathAlignAndShoot(paths[3], SwerveDriveToPose.speakerRelative(-45)),
            new FollowPathAndIntake(paths[4]),
            new FollowPathAlignAndShoot(paths[5], SwerveDriveToPose.speakerRelative(-45))
        );
    }

}