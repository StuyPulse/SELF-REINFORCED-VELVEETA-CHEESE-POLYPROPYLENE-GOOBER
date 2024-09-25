package com.stuypulse.robot;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.stuypulse.robot.commands.leds.LEDReset;
import com.stuypulse.robot.commands.vision.VisionReloadWhiteList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer robot;
    private CommandScheduler scheduler;
    private Command auto;

    public enum State {
        AUTON,
        TELEOP,
        TEST,
        DISABLED
    }

    private static State state;

    /*************************/
    /*** ROBOT SCHEDULEING ***/
    /*************************/

    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStar());

        DataLogManager.start();

        scheduler = CommandScheduler.getInstance();

        robot = new RobotContainer();

        if (Robot.isReal()) CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 80, 60, 30);

        state = State.DISABLED;
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static boolean inTeleop() {
        return state == State.TELEOP;
    }

    /*********************/
    /*** DISABLED MODE ***/
    /*********************/

    @Override
    public void disabledInit() {
        state = State.DISABLED;
        SmartDashboard.putString("Robot State", state.toString());
    }

    @Override
    public void disabledPeriodic() {
        // reload whitelist in case of alliance change
        CommandScheduler.getInstance().schedule(new VisionReloadWhiteList());
    }

    /***********************/
    /*** AUTONOMOUS MODE ***/
    /***********************/  

    @Override
    public void autonomousInit() {
        auto = robot.getAutonomousCommand();

        if (auto != null) {
            auto.schedule();
        }

        scheduler.schedule(new LEDReset());

        state = State.AUTON;
        SmartDashboard.putString("Robot State", state.toString());
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    /*******************/
    /*** TELEOP MODE ***/
    /*******************/

    @Override
    public void teleopInit() {
        if (auto != null) {
            auto.cancel();
        }
        state = State.TELEOP;
        SmartDashboard.putString("Robot State", state.toString());
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    /*****************/
    /*** TEST MODE ***/
    /*****************/

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        state = State.TEST;
        SmartDashboard.putString("Robot State", state.toString());
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
