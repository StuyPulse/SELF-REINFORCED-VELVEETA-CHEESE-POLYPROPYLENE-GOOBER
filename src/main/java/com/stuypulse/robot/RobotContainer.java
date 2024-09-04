package com.stuypulse.robot;

import com.ctre.phoenix6.Utils;
import com.stuypulse.robot.commands.BuzzController;
import com.stuypulse.robot.commands.arm.ArmToAmp;
import com.stuypulse.robot.commands.arm.ArmToClimbing;
import com.stuypulse.robot.commands.arm.ArmToFeed;
import com.stuypulse.robot.commands.arm.ArmToLobFerry;
import com.stuypulse.robot.commands.arm.ArmToLowFerry;
import com.stuypulse.robot.commands.arm.ArmToPreClimb;
import com.stuypulse.robot.commands.arm.ArmToSpeaker;
import com.stuypulse.robot.commands.arm.ArmToSubwooferShot;
import com.stuypulse.robot.commands.arm.ArmWaitUntilAtTarget;
import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.auton.Mobility;
import com.stuypulse.robot.commands.auton.ADEF.FivePieceADEF;
import com.stuypulse.robot.commands.auton.BCA.FourPieceBCA;
import com.stuypulse.robot.commands.intake.IntakeAcquire;
import com.stuypulse.robot.commands.intake.IntakeAcquireForever;
import com.stuypulse.robot.commands.intake.IntakeDeacquire;
import com.stuypulse.robot.commands.intake.IntakeFeed;
import com.stuypulse.robot.commands.intake.IntakeShoot;
import com.stuypulse.robot.commands.intake.IntakeStop;
import com.stuypulse.robot.commands.leds.LEDDefaultMode;
import com.stuypulse.robot.commands.leds.LEDReset;
import com.stuypulse.robot.commands.leds.LEDSet;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntake;
import com.stuypulse.robot.commands.shooter.ShooterAcquireFromIntakeWithRetry;
import com.stuypulse.robot.commands.shooter.ShooterFeederDeacquire;
import com.stuypulse.robot.commands.shooter.ShooterFeederShoot;
import com.stuypulse.robot.commands.shooter.ShooterFeederStop;
import com.stuypulse.robot.commands.shooter.ShooterManualIntake;
import com.stuypulse.robot.commands.shooter.ShooterWaitForTarget;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwerveDriveDriveRobotRelative;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedAmp;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedManualFerry;
import com.stuypulse.robot.commands.swerve.driveAligned.SwerveDriveDriveAlignedSpeaker;
import com.stuypulse.robot.commands.swerve.noteAlignment.SwerveDriveDriveToNote;
import com.stuypulse.robot.commands.swerve.SwerveDriveSeedFieldRelative;
import com.stuypulse.robot.constants.LEDInstructions;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.Telemetry;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.subsystems.vision.NoteVision;
import com.stuypulse.robot.util.PathUtil.AutonConfig;
import com.stuypulse.robot.util.SLColor;
import com.stuypulse.robot.util.ShooterLobFerryInterpolation;
import com.stuypulse.robot.util.ShooterSpeeds;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.leds.LEDController;
import com.stuypulse.robot.subsystems.leds.instructions.LEDInstruction;
import com.stuypulse.robot.subsystems.leds.instructions.LEDPulseColor;
import com.stuypulse.robot.subsystems.leds.instructions.LEDRainbow;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    public final AprilTagVision vision = AprilTagVision.getInstance();
    public final NoteVision noteVision = NoteVision.getInstance();
    
    public final Intake intake = Intake.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Arm arm = Arm.getInstance();
    public final SwerveDrive swerve = SwerveDrive.getInstance();

    public final LEDController leds = LEDController.getInstance();

    private final Telemetry logger = new Telemetry();

    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();

        if (Utils.isSimulation()) {
            swerve.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
        swerve.registerTelemetry(logger::telemeterize);
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new SwerveDriveDrive(driver));
        intake.setDefaultCommand(new IntakeStop());
        leds.setDefaultCommand(new LEDDefaultMode());
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        configureOperatorBindings();
        configureDriverBindings();
    }

    private void configureDriverBindings() {
        driver.getDPadRight().onTrue(new SwerveDriveSeedFieldRelative());

        driver.getDPadUp()
            .onTrue(new ArmToPreClimb())
            .onTrue(new ShooterFeederStop())
            .onTrue(new IntakeStop());

        driver.getDPadDown().onTrue(new ArmToClimbing());

        // intake field relative
        driver.getRightTriggerButton()
            .onTrue(new ArmToFeed())
            // .whileTrue(new SwerveDriveDriveToNote(driver))
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.FIELD_RELATIVE_INTAKING))
                .andThen(new BuzzController(driver))
            );
        
        // intake robot relative
        driver.getLeftTriggerButton()
            .onTrue(new ArmToFeed())
            .whileTrue(new IntakeAcquire()
                .deadlineWith(new LEDSet(LEDInstructions.ROBOT_RELATIVE_INTAKING))
                .andThen(new BuzzController(driver))
            )
            .whileTrue(new SwerveDriveDriveRobotRelative(driver));
        
        // deacquire
        driver.getDPadLeft()
            .whileTrue(new IntakeDeacquire())
            .whileTrue(new ShooterFeederDeacquire())
            .whileTrue(new LEDSet(LEDInstructions.DEACQUIRING));
        
        // speaker align and score 
        // score amp
        driver.getRightBumper()
            .whileTrue(new ConditionalCommand(
                new SwerveDriveDrive(driver)
                    .alongWith(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                        .andThen(new ShooterFeederDeacquire().alongWith(new LEDSet(LEDInstructions.AMP_SCORE)))),
                new SwerveDriveDriveAlignedSpeaker(driver)
                    .alongWith(new ArmToSpeaker()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToSpeaker()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.SPEAKER_ALIGN)),
                () -> Arm.getInstance().getState() == Arm.State.AMP))
            .onFalse(new ShooterFeederStop());

        // lob ferry align and shoot
        driver.getLeftStickButton()
            .whileTrue(new SwerveDriveDriveAlignedFerry(driver)
                    .alongWith(new ArmToLobFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToLobFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN))
            )
            .onFalse(new ShooterFeederStop());


        // low ferry align and shoot
        driver.getRightStickButton()
            .whileTrue(new SwerveDriveDriveAlignedFerry(driver)
                    .alongWith(new ArmToLowFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToLowFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN))
            )
            .onFalse(new ShooterFeederStop());

        // arm to amp
        driver.getLeftBumper().onTrue(new ArmToAmp());

        // manual speaker at subwoofer
        driver.getRightMenuButton()
            .whileTrue(new ArmToSubwooferShot()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new ShooterFeederShoot())
                        )
            .whileTrue(new LEDSet(LEDInstructions.SPEAKER_MANUAL))
            .onFalse(new ShooterFeederStop());
        
        // manual lob ferry
        driver.getTopButton()
            .whileTrue(new SwerveDriveDriveAlignedManualFerry(driver)
                    .alongWith(new ArmToLobFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualLobFerry()))
                        .andThen(new ShooterFeederShoot())
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOB_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ShooterFeederStop());

        // manual low ferry
        driver.getLeftButton()
            .whileTrue(new SwerveDriveDriveAlignedManualFerry(driver)
                    .alongWith(new ArmToLowFerry()
                        .andThen(new ArmWaitUntilAtTarget().withTimeout(Settings.Arm.MAX_WAIT_TO_REACH_TARGET)
                                .alongWith(new ShooterWaitForTarget().withTimeout(Settings.Shooter.MAX_WAIT_TO_REACH_TARGET)))
                        .andThen(new WaitUntilCommand(() -> swerve.isAlignedToManualLowFerry()))
                        .andThen(new ShooterFeederShoot()
                            .alongWith(new IntakeShoot().onlyIf(() -> Arm.getInstance().atIntakeShouldShootAngle()))
                        )
                    )
                    .alongWith(new LEDSet(LEDInstructions.LOW_FERRY_ALIGN_MANUAL))
            )
            .onFalse(new ShooterFeederStop());
        
        // human player attention button
        driver.getRightButton().whileTrue(new LEDSet(LEDInstructions.ATTENTION));
    }

    private void configureOperatorBindings() {

    }

    protected void configureAutomaticCommandScheduling() {
        // automatic handoff
        new Trigger(() -> arm.getState() == Arm.State.FEED 
                    && arm.atTarget() 
                    && !shooter.hasNote()
                    && intake.hasNote()
                    && intake.getState() != Intake.State.DEACQUIRING)
            // .onTrue(new ShooterAcquireFromIntakeWithRetry().andThen(new BuzzController(driver)));
            .onTrue(new ShooterAcquireFromIntake().andThen(new BuzzController(driver)));
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.addOption("Do Nothing", new DoNothingAuton());
        autonChooser.addOption("Mobility", new Mobility());

        AutonConfig BCA = new AutonConfig("4 BCA", FourPieceBCA::new,
            "Center to B", "B to C", "C to A");
        AutonConfig BCA_RED = new AutonConfig("4 BCA RED", FourPieceBCA::new,
        "Center to B", "B to C", "C to A");
        // AutonConfig HGF = new AutonConfig("4 HGF", FourPieceHGF::new,
        // "Source to H", "H to Shoot", "H Shoot to G", "G to Shoot", "G Shoot to F", "F to Shoot");
        // AutonConfig HGF_RED = new AutonConfig("4 HGF RED", FourPieceHGF::new,
        // "Source to H", "H to Shoot", "H Shoot to G", "G to Shoot", "G Shoot to F", "F to Shoot");
        AutonConfig ADEF = new AutonConfig("5 ADEF", FivePieceADEF::new,
        "Amp to A", "A to D", "D to Shoot", "D Shoot to E", "E to Shoot", "E Shoot to F", "F to Shoot");
        AutonConfig ADEF_RED = new AutonConfig("5 ADEF RED", FivePieceADEF::new,
        "Amp to A", "A to D", "D to Shoot", "D Shoot to E", "E to Shoot", "E Shoot to F", "F to Shoot");

        BCA.registerDefaultBlue(autonChooser);
        BCA_RED.registerRed(autonChooser);

        // HGF.registerBlue(autonChooser);
        // HGF_RED.registerRed(autonChooser);

        ADEF.registerBlue(autonChooser);
        ADEF_RED.registerRed(autonChooser);

        SmartDashboard.putData("Autonomous", autonChooser);
        
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    public static String getAutonomousCommandNameStatic() {
        if (autonChooser.getSelected() == null) {
            return "Do Nothing";
        }

        return autonChooser.getSelected().getName();

    }

}
