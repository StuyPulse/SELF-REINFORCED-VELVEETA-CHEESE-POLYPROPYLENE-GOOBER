package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.intake.IntakeSetAcquire;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CenterMobilityWithWait extends SequentialCommandGroup {
    
    public CenterMobilityWithWait(PathPlannerPath... paths) {
        addCommands(
            ShootRoutine.fromSubwoofer(),
            new ArmToFeed(),
            new WaitCommand(10),
            new IntakeSetAcquire(),
            SwerveDrive.getInstance().followPathCommand(paths[0])
        );
    }
}
