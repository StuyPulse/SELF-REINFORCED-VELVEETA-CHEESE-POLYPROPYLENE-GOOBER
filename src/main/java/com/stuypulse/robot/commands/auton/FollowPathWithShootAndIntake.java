package com.stuypulse.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.constants.Settings.Auton;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.commands.shooter.ShooterScoreSpeaker;
import com.stuypulse.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FollowPathWithShootAndIntake extends SequentialCommandGroup {
    
    public FollowPathWithShootAndIntake(PathPlannerPath path, double shootTime) {
        this(path, shootTime, Auton.DEFAULT_INTAKE_TIMEOUT);
    }

    public FollowPathWithShootAndIntake(PathPlannerPath path, double shootTime, double intakeTimeout) {
        addCommands(
            new ParallelCommandGroup(
                SwerveDrive.getInstance().followPathCommand(path)
                    .until(() -> Intake.getInstance().hasNote()),
            new SequentialCommandGroup(
                new WaitCommand(shootTime),
                new ShooterWaitForTarget(),
                new ShooterScoreSpeaker(), 
                new IntakeAcquire()
            )
            )
        );
    }
}