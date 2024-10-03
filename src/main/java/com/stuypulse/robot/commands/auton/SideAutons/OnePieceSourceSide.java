package com.stuypulse.robot.commands.auton.SideAutons;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.auton.ShootRoutine;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OnePieceSourceSide extends SequentialCommandGroup {
    
    public OnePieceSourceSide(PathPlannerPath... paths) {
        addCommands(
            // Preload Shot
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),

            new WaitCommand(10),

            // Mobility
            SwerveDrive.getInstance().followPathCommand(paths[0])
        );
    }

}
