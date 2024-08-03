package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class UntilNoteShot extends SequentialCommandGroup {

    public UntilNoteShot(double timeout) {
        addCommands(
            new InstantCommand(SwerveDrive.getInstance()::stop, SwerveDrive.getInstance()),
            new WaitUntilCommand(Shooter.getInstance()::hasNote)
                .withTimeout(timeout),
            new IntakeStop()
        );
    }
}