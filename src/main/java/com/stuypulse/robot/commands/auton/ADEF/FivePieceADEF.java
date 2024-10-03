package com.stuypulse.robot.commands.auton.ADEF;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.FollowPathThenShoot;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FivePieceADEF extends SequentialCommandGroup {
    
    public FivePieceADEF(PathPlannerPath... paths) {
        
        addCommands(
            // Preload Shot
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to A + Shoot A
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0]),
            ShootRoutine.fromAnywhere(),
            new ArmToFeed(),

            // Drive to D + Shoot D
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[1]),
            new FollowPathThenShoot(paths[2], false),
            new ArmToFeed(),

            // Drive to E + Shoot E
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[3]),
            new FollowPathThenShoot(paths[4], false),
            new ArmToFeed(),

            // Drive to F + Shoot F
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[5])
            // new FollowPathThenShoot(paths[6], true),
            // new ArmToFeed()
        );
    }

}
