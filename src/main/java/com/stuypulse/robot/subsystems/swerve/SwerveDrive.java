package com.stuypulse.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.vision.AprilTagVision;
import com.stuypulse.robot.util.vision.VisionData;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

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
        field = new Field2d();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(0.005);
    }

    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    public Rotation2d getGyroAngle() {
        return Rotation2d.fromRotations(m_yawGetter.getValueAsDouble());
    }

    public SwerveModulePosition[] getModulePositions() {
        return m_modulePositions;
    }

    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    public Field2d getField() {
        return field;
    }

    public void initFieldObject() {
        String[] ids = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < Modules.length; i++) {
            modules2D[i] = field.getObject(ids[i] + "-2d");
        }
    }

    public boolean isAlignedToSpeaker() {
        Translation2d currentPose = SwerveDrive.getInstance().getPose().getTranslation();
        Translation2d speakerPose = Field.getAllianceSpeakerPose().getTranslation();
        Rotation2d targetAngle = speakerPose.minus(currentPose).getAngle();

        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToLowFerry() {
        Rotation2d targetAngle = getPose().getTranslation().minus(Field.getAmpCornerPose()).getAngle();
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToLobFerry() {
        Rotation2d targetAngle = getPose().getTranslation().minus(Field.getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToManualLowFerry() {
        Rotation2d targetAngle = Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle();
        return Math.abs(getPose().getRotation().minus(targetAngle).getDegrees()) < Settings.Alignment.ANGLE_TOLERANCE.get();
    }

    public boolean isAlignedToManualLobFerry() {
        Rotation2d targetAngle = Field.getManualFerryPosition().minus(Field.getAmpCornerPose()).getAngle().plus(Rotation2d.fromDegrees(180));
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

        // addVisionMeasurement(poseSum.div(areaSum), timestampSum / areaSum,
            // DriverStation.isAutonomous() ? VecBuilder.fill(0.9, 0.9, 10) : VecBuilder.fill(0.7, 0.7, 10));
        
        addVisionMeasurement(new Pose2d(3, 3, new Rotation2d()), timestampSum / areaSum,
            DriverStation.isAutonomous() ? VecBuilder.fill(0.9, 0.9, 10) : VecBuilder.fill(0.7, 0.7, 10));
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
    private void applyOperatorPerspective() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    @Override
    public void periodic() {
        String[] moduleIds = {"Front Left", "Front Right", "Back Left", "Back Right"};
        for (int i = 0; i < Modules.length; i++) {
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Target Angle (deg)", Modules[i].getTargetState().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Angle (deg)", Modules[i].getCurrentState().angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Target Velocity (m/s)", Modules[i].getTargetState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Velocity (m/s)", Modules[i].getCurrentState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Angle Error", Modules[i].getTargetState().angle.minus(Modules[i].getCurrentState().angle).getDegrees());

            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Drive Current", Modules[i].getDriveMotor().getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Drive Voltage", Modules[i].getDriveMotor().getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Turn Current", Modules[i].getSteerMotor().getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Swerve/Modules/" + moduleIds[i] + "/Turn Voltage", Modules[i].getSteerMotor().getMotorVoltage().getValueAsDouble());
        }

        field.setRobotPose(getPose());

        applyOperatorPerspective();

        ArrayList<VisionData> outputs = AprilTagVision.getInstance().getOutputs();
        if (Settings.Vision.IS_ACTIVE.get() && outputs.size() > 0) {
            updateEstimatorWithVisionData(outputs);
        }
    }
}