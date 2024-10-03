package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveAlignToSpeaker;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public abstract class ShootRoutine {

    public static Command fromSubwoofer() {
        return new SequentialCommandGroup(
            new ArmToSubwooferShot(),
            new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                .alongWith(new ShooterWaitForTarget().withTimeout(1.0)),
            new ShooterFeederShoot(),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasNote()).alongWith(new WaitCommand(0.5)),
            new ShooterFeederStop()
        );
    }

    public static Command fromAnywhere() {
        return new SequentialCommandGroup(
            new ArmToSpeaker(),
            new ParallelCommandGroup(
                new SwerveDriveAlignToSpeaker(),
                new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET),
                new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)
            ),
            new ShooterFeederShoot(),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasNote()).alongWith(new WaitCommand(1.25)),
            new ShooterFeederStop()
        );
    }

    public static Command fromAnywhereLastShot() {
        return new SequentialCommandGroup(
            new ArmToSpeaker(),
            new ParallelCommandGroup(
                new SwerveDriveAlignToSpeaker(),
            new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET),
                new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)
            ),
            new ShooterFeederShoot(),
            new WaitUntilCommand(() -> !Shooter.getInstance().hasNote())
                .alongWith(new WaitCommand(2.5)),
            new ShooterFeederStop()
        );
    }

    // public static Command fromLastShot() {
    //     return new SequentialCommandGroup(
    //         new ArmToSpeaker().onlyIf(() -> Shooter.getInstance().hasNote()),
    //         new ParallelCommandGroup(
    //             new SwerveDriveAlignToSpeaker(),
    //         new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET),
    //             new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)
    //         ),
    //         new ShooterFeederShoot(),
    //         new WaitUntilCommand(() -> !Shooter.getInstance().hasNote())
    //     );
    // }
}