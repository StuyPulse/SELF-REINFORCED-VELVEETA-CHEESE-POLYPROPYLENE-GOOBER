package com.stuypulse.robot.subsystems.swerve;

import java.util.ArrayList;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Motion;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.FollowPathPointSpeakerCommand;
import com.stuypulse.robot.util.vision.VisionData;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class SwerveDrive extends SwerveDrivetrain implements Subsystem {

    private static final SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            SwerveDriveConstants.DrivetrainConstants,
            SwerveDriveConstants.UpdateOdometryFrequency,
            SwerveDriveConstants.FrontLeft,
            SwerveDriveConstants.FrontRight,
            SwerveDriveConstants.BackLeft,
            SwerveDriveConstants.BackRight
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }

    private final Field2d field;
    private final FieldObject2d[] modules2D;

    private final Translation2d[] moduleOffsets;

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    protected SwerveDrive(SwerveDrivetrainConstants driveTrainConstants, double UpdateOdometryFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, UpdateOdometryFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        modules2D = new FieldObject2d[Modules.length];
        
        moduleOffsets = new Translation2d[] {
            Settings.Swerve.FrontLeft.MODULE_OFFSET,
            Settings.Swerve.FrontRight.MODULE_OFFSET,
            Settings.Swerve.BackLeft.MODULE_OFFSET,
            Settings.Swerve.BackRight.MODULE_OFFSET,
        };

        field = new Field2d();
        initFieldObject();
        SmartDashboard.putData("Field", field);

        configureAutoBuilder();
    }

    /*** PATH FOLLOWING ***/

    public Command followPathCommand(String pathName) {
        return followPathCommand(PathPlannerPath.fromPathFile(pathName));
    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathHolonomic(
            path,
            this::getPose,
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Motion.XY,
                Motion.THETA,
                4.9,
                Math.hypot(Settings.Swerve.LENGTH, Settings.Swerve.WIDTH),
                new ReplanningConfig(false, false)
            ),
            () -> false,
            this
        );
    }

    public Command followPathWithSpeakerAlignCommand(PathPlannerPath path) {
        return new FollowPathPointSpeakerCommand(
            path, 
            this::getPose, 
            this::getChassisSpeeds, 
            this::setChassisSpeeds, 
            new PPHolonomicDriveController(
                Motion.XY, 
                Motion.THETA, 
                0.02, 
                4.9, 
                Math.hypot(Settings.Swerve.LENGTH, Settings.Swerve.WIDTH)),
            new ReplanningConfig(false, false),
            () -> false,
            this
        );
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(m_moduleStates);
    }

    public void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
        SmartDashboard.putNumber("Swerve/Chassis Target X", robotSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Y", robotSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Target Omega", robotSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds speeds = new ChassisSpeeds(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, -robotSpeeds.omegaRadiansPerSecond);
        setControl(drive.withSpeeds(speeds));
    }

    public void drive(Vector2D velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            Robot.isBlue() ? velocity.y : -velocity.y, 
            Robot.isBlue() ? -velocity.x : velocity.x,
            -rotation,
            getPose().getRotation());

        Pose2d robotVel = new Pose2d(
            Settings.DT * speeds.vxMetersPerSecond,
            Settings.DT * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(Settings.DT * speeds.omegaRadiansPerSecond));
        Twist2d twistVel = new Pose2d().log(robotVel);

        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / Settings.DT,
            twistVel.dy / Settings.DT,
            twistVel.dtheta / Settings.DT
        ));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(Settings.DT, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(0.005);
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRotations(m_yawGetter.getValueAsDouble());
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public void configureAutoBuilder() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            (Pose2d pose) -> seedFieldRelative(pose),
            this::getChassisSpeeds,
            this::setChassisSpeeds,
            new HolonomicPathFollowerConfig(
                Settings.Swerve.Motion.XY,
                Settings.Swerve.Motion.THETA,
                4.9,
                Math.hypot(Settings.Swerve.LENGTH, Settings.Swerve.WIDTH),
                new ReplanningConfig(true, true)),
            () -> false,
            instance
        );

        PathPlannerLogging.setLogActivePathCallback((poses) -> getField().getObject("path").setPoses(poses));
    }

    public void initFieldObject() {
        String[] ids = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < Modules.length; i++) {
            modules2D[i] = field.getObject(ids[i] + "-2d");
        }
    }

    public boolean isMoving() {
        return Math.hypot(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond) 
                > (Settings.Driver.Drive.DEADBAND.get() * Settings.Swerve.MAX_LINEAR_VELOCITY);
    }

    public boolean isAlignedToSpeaker() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        // Rotate by 180 because the shooter is on the back of the robot
        Rotation2d targetAngle = speakerPose.minus(currentPose).getAngle().rotateBy(Rotation2d.fromDegrees(180));
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToFerry() {
        Rotation2d targetAngle = getPose().getTranslation().minus(Field.getAmpCornerPose()).getAngle();
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToManualFerry() {
        Rotation2d targetAngle = Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle();
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    private void updateEstimatorWithVisionData(ArrayList<VisionData> outputs) {
        Pose2d poseSum = new Pose2d();
        double timestampSum = 0;
        double areaSum = 0;

        for (VisionData data : outputs) {
            Pose2d weighted = data.getPose().toPose2d().times(data.getArea());

            poseSum = new Pose2d(
                poseSum.getTranslation().plus(weighted.getTranslation()),
                poseSum.getRotation().plus(weighted.getRotation())
            );

            areaSum += data.getArea();

            timestampSum += data.getTimestamp() * data.getArea();
        }

        addVisionMeasurement(poseSum.div(areaSum), timestampSum / areaSum,
            DriverStation.isAutonomous() ? VecBuilder.fill(0.2, 0.2, 1) : VecBuilder.fill(0.2, 0.2, 1));
    }

    public void setVisionEnabled(boolean enabled) {
        Settings.Vision.IS_ACTIVE.set(enabled);
    }

    /**
     * Try to apply the operator perspective 
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state 
     * This allows us to correct the perspective in case the robot code restarts mid-match 
     * Otherwise, only check and apply the operator perspective if the DS is disabled
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing
     * 
     * <p>Should call this periodically
     */
    // private void applyOperatorPerspective() {
    //     if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //         DriverStation.getAlliance().ifPresent((allianceColor) -> {
    //             this.setOperatorPerspectiveForward(
    //                     allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
    //                             : BlueAlliancePerspectiveRotation);
    //             hasAppliedOperatorPerspective = true;
    //         });
    //     }
    // }

    @Override
    public void periodic() {
        String[] moduleIds = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < Modules.length; i++) {
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Target Angle (deg)", Modules[i].getTargetState().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Angle (deg)", Modules[i].getCurrentState().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Target Velocity (meters per s)", Modules[i].getTargetState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Velocity (meters per s)", Modules[i].getCurrentState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Angle Error", Modules[i].getTargetState().angle.minus(Modules[i].getCurrentState().angle).getDegrees());

            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Drive Current", Modules[i].getDriveMotor().getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Drive Voltage", Modules[i].getDriveMotor().getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Turn Current", Modules[i].getSteerMotor().getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Turn Voltage", Modules[i].getSteerMotor().getMotorVoltage().getValueAsDouble());
        }

        field.setRobotPose(getPose());

        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();
        if (Settings.Vision.IS_ACTIVE.get() && outputs.size() > 0) {
            updateEstimatorWithVisionData(outputs);
        }

        for (int i = 0; i < Modules.length; i++) {
            modules2D[i].setPose(new Pose2d(
                getPose().getTranslation().plus(moduleOffsets[i].rotateBy(getPose().getRotation())),
                getModule(i).getCurrentState().angle.plus(getPose().getRotation())
            ));
        }
    }
}