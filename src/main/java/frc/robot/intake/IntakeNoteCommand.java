// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.LimelightConstants.DetectionConstants;
import frc.robot.limelights.DetectionSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.TunerConstants;
import frc.robot.utilities.CommandGenerators;

/** A command that orbits a note while driving and intakes it. */
public class IntakeNoteCommand extends Command {
    private final double MaxAngularRate = TunerConstants.kAngularSpeedMaxRadps;
    private final double reasonableMaxAngularRate;
    private final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private final double reasonableMaxSpeed;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> angularSpeedSupplier;

    private final Supplier<Boolean> uncappedSpeedSupplier;
    private final Supplier<Boolean> fineControlSupplier;

    private Command intakeCommand;
    private boolean executeIntakeCommand;
    Pose2d lastNotePose;
    private boolean facingBlue;
    private boolean noAlliance;

    /**
     * Creates a new IntakeNoteCommand.
     * @param xSpeedSupplier - Supplier for the x-speed of the robot.
     * @param ySpeedSupplier - Supplier for the y-speed of the robot.
     * @param angularSpeedSupplier - Supplier for the angular speed of the robot.
     * @param uncappedSpeedSupplier - Supplier for driving at top speed.
     * @param fineControlSupplier - Supplier for driving with fine control speed.
     * @param center - Whether or not to center on the note.
     */
    public IntakeNoteCommand(
        Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier,
        Supplier<Double> angularSpeedSupplier, 
        Supplier<Boolean> uncappedSpeedSupplier, Supplier<Boolean> fineControlSupplier
    ) {
        setName("IntakeNoteCommand");

        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.angularSpeedSupplier = angularSpeedSupplier;

        this.uncappedSpeedSupplier = uncappedSpeedSupplier;
        this.fineControlSupplier = fineControlSupplier;

        this.reasonableMaxSpeed = TunerConstants.reasonableMaxSpeed;
        this.reasonableMaxAngularRate = TunerConstants.reasonableMaxAngularRate;

        this.intakeCommand = CommandGenerators.IntakeCommand();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(CommandSwerveDrivetrain.getInstance(), IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.lastNotePose = null;
        this.executeIntakeCommand = false;
        this.noAlliance = false;

        try {
            this.facingBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        }
        catch (NoSuchElementException e) {
            System.err.println("Alliance is empty ; cannot target NOTE to set rotation addition.");
            this.noAlliance = true;
            return;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.noAlliance) return;

        boolean topSpeed = this.uncappedSpeedSupplier.get();
        boolean fineControl = this.fineControlSupplier.get();

        double velocityX = this.xSpeedSupplier.get()
            * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);
        double velocityY = this.ySpeedSupplier.get()
            * (topSpeed ? MaxSpeed : reasonableMaxSpeed)
            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);

        Pose2d[] notePoses = DetectionSubsystem.getInstance().getRecentNotePoses();
        Translation2d botTranslation = CommandSwerveDrivetrain.getInstance().getState().Pose.getTranslation();
        
        // If no Notes OR Note further than {@link DetectionConstants#MAX_NOTE_DISTANCE_DRIVING} meters, drive normally and intake.
        if (notePoses.length == 0
            || botTranslation.getDistance(notePoses[0].getTranslation())
                >= DetectionConstants.MAX_NOTE_DISTANCE_DRIVING
            || (
                this.lastNotePose == null
                && botTranslation.getDistance(notePoses[0].getTranslation()) <= 1
            )
            || (
                this.lastNotePose != null
                && botTranslation.getDistance(this.lastNotePose.getTranslation()) <= 0.5
            )
        ) {
            if (this.lastNotePose != null
                && botTranslation.getDistance(this.lastNotePose.getTranslation()) <= 0.5
            ) {
                this.lastNotePose = null;
            }
            final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest.FieldCentric()
                .withDeadband(reasonableMaxSpeed * ControllerConstants.DEADBAND)
                .withRotationalDeadband(reasonableMaxAngularRate * ControllerConstants.DEADBAND) // Add a deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

            if (!this.executeIntakeCommand) {
                this.intakeCommand.initialize();
                this.executeIntakeCommand = true;
            }
            
            CommandSwerveDrivetrain.getInstance().setControl(
                fieldCentricDrive_withDeadband
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withRotationalRate(
                        this.angularSpeedSupplier.get()
                        * (topSpeed ? MaxAngularRate : reasonableMaxAngularRate)
                        * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1)
                    )
            );
        }
        else {
            final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
                .withDeadband(reasonableMaxSpeed * ControllerConstants.DEADBAND)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
            // If within one meter, enable the intake and stop updating note pose because it might be faulty data.
            if (this.lastNotePose != null && botTranslation.getDistance(this.lastNotePose.getTranslation()) <= 1.5) {
                if (!this.executeIntakeCommand) {
                    this.intakeCommand.initialize();
                    this.executeIntakeCommand = true;
                }
            }
            else {
                this.lastNotePose = notePoses[0];
            }

            Rotation2d targetRotation = new Rotation2d(Math.atan2(
                this.lastNotePose.getY() - botTranslation.getY(),
                this.lastNotePose.getX() - botTranslation.getX()
            ) + (this.facingBlue ? 0 : Math.PI));

            CommandSwerveDrivetrain.getInstance().setControl(
                fieldCentricFacingAngle_withDeadband
                    .withVelocityX(velocityX)
                    .withVelocityY(velocityY)
                    .withTargetDirection(targetRotation)
            );
        }

        if (this.executeIntakeCommand) {
            this.intakeCommand.execute();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.intakeCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.noAlliance || this.intakeCommand.isFinished();
    }
}
