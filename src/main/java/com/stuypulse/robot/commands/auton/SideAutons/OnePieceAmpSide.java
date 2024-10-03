package com.stuypulse.robot.commands.auton.SideAutons;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class OnePieceAmpSide extends SequentialCommandGroup {
    
    public OnePieceAmpSide(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new WaitCommand(10),

            new IntakeSetAcquire(),
            // Mobility
            SwerveDrive.getInstance().followPathCommand(paths[0])
            // new WaitUntilCommand(() -> Shooter.getInstance().hasNote()).andThen(ShootRoutine.fromAnywhere())
            //     .onlyIf(() -> Intake.getInstance().hasNote() || Shooter.getInstance().hasNote())
        );
    }

}
