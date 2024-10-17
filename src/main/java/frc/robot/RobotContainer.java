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
import frc.robot.intake.IntakeSubsystem;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.limelights.VisionSubsystem;
import frc.robot.pivot.ManuallyPivotCommand;
import frc.robot.pivot.PivotCommand;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.pivot.ResetAtHardstopCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.Telemetry;
import frc.robot.swerve.TunerConstants;
import frc.robot.constants.Constants.BehaviorConstants;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.LimelightConstants.DetectionConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.auto.DriveToNoteCommand;
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
        .withSize(2, 3)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"));

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
        final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
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
        Command intakeCommand = CommandGenerators.getIntakeCommand();
        
        this.driverController.leftBumper()
            .whileTrue(drivetrain.applyRequest(() -> {
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

                if (IntakeSubsystem.getInstance().backLaserHasNote() && !IntakeSubsystem.getInstance().frontLaserHasNote()) {
                    intakeCommand.schedule();
                }
                
                // If no Notes OR Note further than {@link DetectionConstants#MAX_NOTE_DISTANCE_DRIVING} meters, drive normally.
                if (notePoses.length == 0
                    || notePoses[0].getTranslation().getDistance(botTranslation)
                        >= DetectionConstants.MAX_NOTE_DISTANCE_DRIVING
                    || drivetrain.getState().Pose.getTranslation().getDistance(notePoses[0].getTranslation())
                        <= 1
                ) {
                    if (!intakeCommand.isScheduled()) {
                        intakeCommand.schedule();
                    }
                    
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
                    
                    // If within one meter, enable the intake
                    if (drivetrain.getState().Pose.getTranslation().getDistance(notePose.getTranslation())
                                <= 1
                            && !intakeCommand.isScheduled()
                    ) {
                        intakeCommand.schedule();
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
            }))
            .onFalse(Commands.runOnce(
                () -> CommandScheduler.getInstance().cancel(intakeCommand)
            ));
        
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

                Pose2d botPose = drivetrain.getState().Pose;
                Rotation2d targetRotation = new Rotation2d(
                    Math.atan2(
                        speakerTranslation.getY() - botPose.getY(),
                        speakerTranslation.getX() - botPose.getX()
                    ) + Math.PI // Face with shooter, which is the back of the bot
                );

                double goalAngle = targetRotation.getDegrees();
                double currentAngle = botPose.getRotation().getDegrees();

                if (Math.min(Math.abs(goalAngle - currentAngle), 360 - Math.abs(goalAngle - currentAngle))
                        <= BehaviorConstants.FACING_ANGLE_TOLERANCE)
                    {
                        System.out.println("FacingSpeaker | Ready to shoot.");
                    }
                
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
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(0, 0);
        // Re-set chosen position.
        this.layout.add("Set Starting Position",
            Commands.runOnce(
                () -> drivetrain.seedFieldRelative(Positions.getStartingPose(this.positionChooser.getSelected()))
            ).ignoringDisable(true).withName("Set Again"))
            .withWidget(BuiltInWidgets.kCommand)
            .withPosition(0, 1);
    }

    /** Creates instances of each subsystem so periodic runs */
    private void initializeSubsystems() {
        VisionSubsystem.getInstance();
        DetectionSubsystem.getInstance();

        IntakeSubsystem.getInstance();
        PivotSubsystem.getInstance();
        ShooterSubsystem.getInstance();
    }

    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        
    }

    /**
     * Configures the button bindings of the driver controller
     * @apiNote POV, joysticks, and start / back are all used in {@link RobotContainer#configureDrivetrain()}
     */
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
        // this.driverController.back().onTrue(
        //     CommandSwerveDrivetrain.getInstance().runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldRelative(
        //         new Pose2d(new Translation2d(0.5, 5), new Rotation2d())
        //     ))
        // );
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
         *                           Freely rotate within 1 meter.
         *     Right bumper (hold) : Targets SPEAKER to rotate around.
         *                           Does NOT shoot (operator's job).
         */
        this.driverController.x().onTrue(Commands.deadline(
            CommandGenerators.getIntakeCommand(),
            new DriveToNoteCommand()
        ));
    }

    /** Configures the button bindings of the driver controller */
    private void configureOperatorBindings() {
        // Cancel all currently scheduled commands
        operatorController.b().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        PivotSubsystem.getInstance().setDefaultCommand(new ManuallyPivotCommand(
            () -> operatorController.getRightTriggerAxis(),
            () -> operatorController.getLeftTriggerAxis(),
            false
        ));
        operatorController.a().onTrue(new ResetAtHardstopCommand(true));

        operatorController.pov(0)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(90)
            ));
        operatorController.pov(180)
            .whileTrue(PivotSubsystem.getInstance().run(
                () -> PivotSubsystem.getInstance().motionMagicPosition(5)
            ));

        // Testing shooting
        operatorController.pov(90).whileTrue(IntakeSubsystem.getInstance().runEnd(
            () -> IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.SLOW_INTAKE_VELOCITY),
            () -> IntakeSubsystem.getInstance().setSpeed(0)
        ));
        operatorController.pov(270).whileTrue(Commands.runEnd(
            () -> {
                ShooterSubsystem.getInstance().motionMagicVelocity(-5);
                IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_EJECT_VELOCITY);
            },
            () -> {
                ShooterSubsystem.getInstance().setSpeed(0);
                IntakeSubsystem.getInstance().setSpeed(0);
            },
            ShooterSubsystem.getInstance(), IntakeSubsystem.getInstance()
        ));
        
        operatorController.rightBumper()
            .whileTrue(Commands.sequence(
                new PivotCommand(BehaviorConstants.PIVOT_POSITION_SPEAKER),
                new ShootCommand(25)
            ))
            .onFalse(new PivotCommand(PivotConstants.ABOVE_LIMELIGHT_ANGLE));
        
        operatorController.leftBumper()
            // TODO : Account for drop
            .onTrue(Commands.sequence(
                new PivotCommand(10, true, true),
                new ShootCommand(30)
            ))
            .onFalse(Commands.parallel(
                    new PivotCommand(PivotConstants.ABOVE_LIMELIGHT_ANGLE),
                    Commands.runOnce(
                        () -> {},
                        ShooterSubsystem.getInstance(),
                        IntakeSubsystem.getInstance()
                    )
                ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}