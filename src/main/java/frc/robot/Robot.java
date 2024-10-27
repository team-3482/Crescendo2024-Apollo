// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.LimelightConstants;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.utilities.BuildConstants; // If this file does not exist, build the robot code.

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command auton;
    // private PowerDistribution pdh; // TODO : PDH Voltage display
    
    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @SuppressWarnings("resource")
    @Override
    public void robotInit() {
        // Port forward all required LL ports. Necessary for robot connections over ethernet.
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, LimelightConstants.FRONT_APRIL_TAG_LL + ".local", port);
            PortForwarder.add(port + 10, LimelightConstants.NOTE_DETECTION_LL + ".local", port);
            PortForwarder.add(port + 20, LimelightConstants.BACK_APRIL_TAG_LL + ".local", port);
        }

        // record buildconstants data in telemetry
        Logger.recordMetadata("ProjectName", "Crescendo2024-Bot-2");
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Dirty", String.valueOf(BuildConstants.DIRTY));

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
        }
        else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
    
        // TODO LOGGING : This is spamming the console "DataLog no file found"
        // Logger.start();

        // Initialize RobotContainer and all subsystems
        RobotContainer container = RobotContainer.getInstance();
        container.configureDriverBindings();
        container.configureOperatorBindings();

        LiveWindow.disableAllTelemetry();

        // this.pdh = new PowerDistribution(0, ModuleType.kRev); // TODO : PDH Voltage display
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();
        if (this.auton != null) {
            try {
                this.auton.schedule();
            }
            catch (Exception error) {
                error.printStackTrace();
            }
        }
        else {
            System.err.println("No auton command found.");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (this.auton != null) {
            this.auton.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
