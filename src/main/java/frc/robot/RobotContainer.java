// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Positions.PositionInitialization;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.limelights.VisionSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.Telemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Positions;

public class RobotContainer {
    // Thread-safe singleton design pattern.
    private static volatile RobotContainer instance;
    private static Object mutex = new Object();


    public static RobotContainer getInstance() {
        RobotContainer result = instance;
       
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new RobotContainer();
                }
            }
        }
        return instance;
    }

    private final SendableChooser<Command> autoChooser;
    // Position chooser
    private final SendableChooser<PositionInitialization> positionChooser = new SendableChooser<PositionInitialization>();
    private final ShuffleboardLayout layout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT).getLayout("SwerveSubsystem", BuiltInLayouts.kList);

    // Instance of the controllers used to drive the robot
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    /** Creates an instance of the robot controller */
    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems

        initializeSubsystems();
        // Register named commands for pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        configureDriverBindings();
        configureOperatorBindings();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(6, 1);
    }

    /**
     * This function initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the {@code Phoenix6 Swerve Example} that can be found on GitHub.
     */
    private void configureDrivetrain() {
        final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
        final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        final double MaxAngularRate = TunerConstants.kAngularSpeedMaxRadps;
        final double reasonableMaxSpeed = MaxSpeed * 0.5;
        final double reasonableMaxAngularRate = MaxAngularRate * 0.5;

        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest.FieldCentric()
            .withDeadband(reasonableMaxSpeed * ControllerConstants.DEADBAND)
            .withRotationalDeadband(reasonableMaxAngularRate * ControllerConstants.DEADBAND) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final Telemetry logger = new Telemetry(MaxSpeed);

        Trigger leftTrigger = this.driverController.leftTrigger();
        Trigger rightTrigger = this.driverController.rightTrigger();

        // Drivetrain will execute this command periodically
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.getAsBoolean();
                boolean fineControl = rightTrigger.getAsBoolean();
                
                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(
                        -driverController.getLeftY()
                        * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
                    // Drive left with negative X (left)
                    .withVelocityY(
                        -driverController.getLeftX()
                        * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(
                        -driverController.getRightX()
                        * (topSpeed ? MaxAngularRate : reasonableMaxAngularRate)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    );
            }).ignoringDisable(true)
        );

        // Toggle intake mode
        // Faces closest note in vision and enables intake within 2 meters,
        // or drives normally with intake enabled when no notes are found.
        final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
            .withDeadband(reasonableMaxSpeed * ControllerConstants.DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        this.driverController.leftBumper().whileTrue(
            drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.getAsBoolean();
                boolean fineControl = rightTrigger.getAsBoolean();

                double velocityX = -driverController.getLeftY()
                    * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                    * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);
                double velocityY = -driverController.getLeftX()
                    * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                    * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);

                Pose2d[] notePoses = DetectionSubsystem.getInstance().getRecentNotePoses();
                Translation2d botTranslation = drivetrain.getState().Pose.getTranslation();
                
                // If no Notes OR Note further than 3 meters, drive normally
                // TODO 2 : Test no distance limit
                if (notePoses.length == 0
                    || notePoses[0].getTranslation().getDistance(botTranslation) >= 3
                ) {
                    System.out.println("IntakeSubsystem Enabled (no note)");
                    return fieldCentricDrive_withDeadband
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(
                            -driverController.getRightX()
                            * (topSpeed ? MaxAngularRate : reasonableMaxAngularRate)
                            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                        );
                }
                else {
                    Pose2d notePose = notePoses[0];
                    
                    // If within two meters, enable the intake
                    if (drivetrain.getState().Pose.getTranslation()
                        .getDistance(notePose.getTranslation()) <= 1
                    ) {
                        System.out.println("IntakeSubsystem Enabled (close to note)");
                    }

                    Rotation2d targetRotation = new Rotation2d(Math.atan2(
                        notePose.getY() - botTranslation.getY(),
                        notePose.getX() - botTranslation.getX()
                    ));

                    return fieldCentricFacingAngle_withDeadband
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withTargetDirection(targetRotation);
                }
            })
        );
        
        this.driverController.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> {
                boolean topSpeed = leftTrigger.getAsBoolean();
                boolean fineControl = rightTrigger.getAsBoolean();

                double velocityX = -driverController.getLeftY()
                    * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                    * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);
                double velocityY = -driverController.getLeftX()
                    * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
                    * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);

                Translation2d speakerTranslation;
                try {
                    speakerTranslation = Positions.getSpeakerTarget().toTranslation2d();
                }
                catch (RuntimeException e) {
                    System.err.println("Alliance is empty ; cannot target SPEAKER.");
                    return fieldCentricDrive_withDeadband
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(
                            -driverController.getRightX()
                            * (topSpeed ? MaxAngularRate : reasonableMaxAngularRate)
                            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                        );
                }

                Translation2d botTranslation = drivetrain.getState().Pose.getTranslation();
                Rotation2d targetRotation = new Rotation2d(
                    Math.atan2(
                        speakerTranslation.getY() - botTranslation.getY(),
                        speakerTranslation.getX() - botTranslation.getX() 
                    ) + Math.PI // Face with shooter, which is the back of the bot
                );
                
                return fieldCentricFacingAngle_withDeadband
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withTargetDirection(targetRotation);
            })
        );

        // Useful for testing
        // final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // this.driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
        // this.driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(
        //         Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
        //         Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
        //     )
        // )));

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        // This looks terrible, but I can't think of a better way to do it </3
        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /** POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Integer[]{ 1,  0}),
                Map.entry( 45, new Integer[]{ 1, -1}),
                Map.entry( 90, new Integer[]{ 0, -1}),
                Map.entry(135, new Integer[]{-1, -1}),
                Map.entry(180, new Integer[]{-1,  0}),
                Map.entry(225, new Integer[]{-1,  1}),
                Map.entry(270, new Integer[]{ 0,  1}),
                Map.entry(315, new Integer[]{ 1,  1})
            );

            povSpeeds.forEach(
                (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                    drivetrain.applyRequest(() -> {
                        boolean faster = leftTrigger.getAsBoolean();
                        boolean robotCentric = rightTrigger.getAsBoolean();
                        
                        return robotCentric
                            ? robotCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                            : fieldCentricDrive
                                .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                                .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                    })
                )
            );
        }

        // Burger
        this.driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
        // Double Rectangle
        this.driverController.back().onTrue(drivetrain.runOnce(() -> {
            Pose2d estimatedPose = VisionSubsystem.getInstance().getEstimatedPose();
            if (!estimatedPose.equals(new Pose2d())) {
                drivetrain.seedFieldRelative(estimatedPose);
            }
        }));
        
        drivetrain.registerTelemetry(logger::telemeterize);

        // Position chooser
        for (PositionInitialization position : PositionInitialization.values()) {
            this.positionChooser.addOption(position.name(), position);
            if (position == PositionInitialization.LIMELIGHT) {
                this.positionChooser.setDefaultOption(position.name(), position);
            }
        }
        
        this.positionChooser.onChange((PositionInitialization position) ->
            drivetrain.seedFieldRelative(Positions.getStartingPose(position))
        );

        this.layout.add("Starting Position", this.positionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser);
        // Re-set chosen position.
        this.layout.add("Set Starting Position",
            Commands.runOnce(
                () -> drivetrain.seedFieldRelative(Positions.getStartingPose(this.positionChooser.getSelected()))
            ).ignoringDisable(true).withName("Set Again"))
            .withWidget(BuiltInWidgets.kCommand);
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {
        VisionSubsystem.getInstance();
        DetectionSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        
    }

    /** Configures the button bindings of the driver controller */
    private void configureDriverBindings() {
        // Cancel all currently scheduled commands
        this.driverController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        /*
         * POV, joysticks, and start/back are all used in configureDrivetrain()
         *           Left joystick : Translational movement
         *          Right joystick : Rotational movement
         *          Start / Burger : Reset heading
         * Back / Double Rectangle : Reset position to LL data (if not empty)
         *    POV (overrides joys) : Directional movement -- 0.25 m/s
         */
        this.driverController.back().onTrue(
            TunerConstants.DriveTrain.runOnce(() -> TunerConstants.DriveTrain.seedFieldRelative(
                new Pose2d(new Translation2d(5, 5), new Rotation2d())
            ))
        );
        /*
         * Triggers are also used in configureDrivetrain()
         *      Left Trigger > 0.5 : Use TOP SPEED for joysticks
         *                           Use 1.5 m/s for POV
         *     Right Trigger > 0.5 : Use FINE CONTROL for joysticks
         *                           Use ROBOT CENTRIC for POV 
         */
        /*
         * Bumpers are also used in configureDrivetrain()
         *      Left bumper (hold) : Targets nearest Note to rotate around.
         *                           Enables intake if no note is seen or if
         *                           within 2 meters of the nearest one.
         *     Right bumper (hold) : Targets SPEAKER to rotate around.
         *                           Does NOT shoot (operator's job).
         */

        // TODO 3 : Test driving to a note with a variety of distances / configurations.
        this.driverController.x().onTrue(new DriveToNoteCommand());
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        // Unused while building and testing new kraken swerve drive
        
        // Cancel all currently scheduled commands
        // operatorController.b().onTrue(Commands.runOnce(() -> {
        //     CommandScheduler.getInstance().cancelAll();
        // }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}