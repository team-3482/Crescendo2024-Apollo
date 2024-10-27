// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Positions.PositionInitialization;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.limelights.VisionSubsystem;
import frc.robot.pivot.ManuallyPivotCommand;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.pivot.ResetAtHardstopCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.Telemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.Positions;
import frc.robot.utilities.CommandGenerators;

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
    private final ShuffleboardLayout layout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("SwerveSubsystem", BuiltInLayouts.kList)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(2, 3)
        .withPosition(0, 4);

    // Instance of the controllers used to drive the robot
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    /** Creates an instance of the robot controller */
    public RobotContainer() {
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain(); // This is done separately because it works differently from other Subsystems
        configurePositionChooser();

        initializeSubsystems();
        // Register named commands for Pathplanner (always do this after subsystem initialization)
        registerNamedCommands();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
            .add("Auto Chooser", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(4, 3)
            .withPosition(2, 4);
    }

    /**
     * This function initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the {@code Phoenix6 Swerve Example} that can be found on GitHub.
     */
    private void configureDrivetrain() {
        final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
        final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        final double MaxAngularRate = TunerConstants.kAngularSpeedMaxRadps;
        final double reasonableMaxSpeed = TunerConstants.reasonableMaxSpeed;
        final double reasonableMaxAngularRate = TunerConstants.reasonableMaxAngularRate;

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
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Configures the PositionInitialization chooser for the odometry. */
    private void configurePositionChooser() {
        // Position chooser
        for (PositionInitialization position : PositionInitialization.values()) {
            this.positionChooser.addOption(position.name(), position);
            if (position == PositionInitialization.MIDDLE) {
                this.positionChooser.setDefaultOption(position.name(), position);
            }
        }
        
        this.positionChooser.onChange((PositionInitialization position) ->
            CommandSwerveDrivetrain.getInstance().seedFieldRelative(Positions.getStartingPose(position))
        );

        this.layout.add("Starting Position", this.positionChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0);
        // Re-set chosen position.
        this.layout.add(
            "Set Starting Position",
            Commands.runOnce(
                () -> CommandSwerveDrivetrain.getInstance()
                    .seedFieldRelative(Positions.getStartingPose(this.positionChooser.getSelected()))
            ).ignoringDisable(true).withName("Set Again"))
            .withWidget(BuiltInWidgets.kCommand)
            .withPosition(0, 1);
    }

    /** Creates instances of each subsystem so periodic always runs. */
    private void initializeSubsystems() {
        VisionSubsystem.getInstance();
        DetectionSubsystem.getInstance();

        IntakeSubsystem.getInstance();
        PivotSubsystem.getInstance();
        ShooterSubsystem.getInstance();

        IntakeSubsystem.getInstance().setDefaultCommand(
            IntakeSubsystem.getInstance().runOnce(
                () -> IntakeSubsystem.getInstance().setSpeed(0)
            )
        );
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("AutonIntakeNote", CommandGenerators.AutonIntakeNoteCommand());
        NamedCommands.registerCommand("AutoShootNoteStaticAnyDistance", CommandGenerators.AutoShootNoteStaticAnyDistanceCommand());
        NamedCommands.registerCommand("ShootSpeakerUpClose", CommandGenerators.ShootSpeakerUpCloseCommand());
        NamedCommands.registerCommand("Intake", CommandGenerators.IntakeCommand());
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        this.driverController.b().onTrue(CommandGenerators.CancelAllCommands());

        /*
         * POV, joysticks, and start/back are all used in configureDrivetrain()
         *           Left joystick : Translational movement
         *          Right joystick : Rotational movement
         *    POV (overrides joys) : Directional movement -- 0.25 m/s
         */
        // Burger
        this.driverController.start().onTrue(CommandGenerators.SetForwardDirectionCommand());
        // Double Rectangle
        this.driverController.back().onTrue(CommandGenerators.ResetPoseUsingLimelightCommand());
        /*
         * Triggers are also used in configureDrivetrain()
         *      Left Trigger > 0.5 : Use TOP SPEED for joysticks
         *                           Use 1.5 m/s for POV
         *     Right Trigger > 0.5 : Use FINE CONTROL for joysticks
         *                           Use ROBOT CENTRIC for POV 
         */
        /*
         *      Left bumper (hold) : Targets nearest Note to rotate around.
         *                           Enables intake if no note is seen or if
         *                           within 2 meters of the nearest one.
         *                           Freely rotate within 1 meter.
         */
        this.driverController.leftBumper()
            .whileTrue(CommandGenerators.CenterAndIntakeNoteCommand());
        this.driverController.rightBumper()
            .whileTrue(CommandGenerators.AutoShootNoteStaticCheckDistanceCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
        
        this.driverController.y()
            .whileTrue(CommandGenerators.AutoShootNoteMovementCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
        // Drive to a note and intake
        this.driverController.x().onTrue(CommandGenerators.AutonIntakeNoteCommand());
        // Manual intake
        this.driverController.a().whileTrue(CommandGenerators.IntakeCommand());
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        this.operatorController.b().onTrue(CommandGenerators.CancelAllCommands());

        PivotSubsystem.getInstance().setDefaultCommand(new ManuallyPivotCommand(
            () -> operatorController.getRightTriggerAxis(),
            () -> operatorController.getLeftTriggerAxis(),
            false
        ));
        this.operatorController.a().onTrue(new ResetAtHardstopCommand(false).withTimeout(5));

        this.operatorController.pov(0)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(90)
            ));
            this.operatorController.pov(180)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(5)
            ));

        // Testing shooting
        this.operatorController.pov(90).whileTrue(CommandGenerators.ManualIntakeCommand());
        this.operatorController.pov(270).whileTrue(CommandGenerators.ManuallyReverseIntakeCommand());
        
        this.operatorController.rightBumper()
            .whileTrue(CommandGenerators.ShootSpeakerUpCloseCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
        
        this.operatorController.leftBumper()
            .whileTrue(CommandGenerators.ShootAmpUpCloseCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
            
        this.operatorController.x()
            .whileTrue(CommandGenerators.AutoPassNoteCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
            
        this.operatorController.y()
            .whileTrue(CommandGenerators.ManuallyPassNoteCommand())
            .onFalse(CommandGenerators.PivotToIdlePositionCommand());
    }

    /**
     * Gets the instance of the driverController.
     * @return The driver controller.
     */
    public CommandXboxController getDriverController() {
        return this.driverController;
    }

    /**
     * Gets the instance of the operatorController.
     * @return The operator controller.
     */
    public CommandXboxController getOperatorController() {
        return this.operatorController;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}