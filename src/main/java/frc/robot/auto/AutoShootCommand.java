// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.TunerConstants;
import frc.robot.utilities.ShotVector;

/** Shoots a Note autonomously. */
public class AutoShootCommand extends Command {
    // Driving while shooting
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Boolean> uncappedSupplier;
    private final Supplier<Boolean> fineControlSupplier;
    private final double reasonableMaxSpeed;

    private Translation3d speakerTranslation3d;
    private ShotVector idealShotVector;
    private boolean allowMovement;
    
    private boolean endEarly;
    private Timer timer;

    final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * Creates a new AutoShootCommand.
     * @param xSupplier - Supplier for x robot movement from -1.0 to 1.0.
     * @param ySupplier - Supplier for y robot movement from -1.0 to 1.0.
     * @param uncappedSupplier - Supplier for uncapped top speed.
     * @param fineControlSupplier - Supplier for fine control.
     * @param allowMovement - Whether to require the driving subsystem, which prevents the driver from moving.
     */
    public AutoShootCommand(
        Supplier<Double> xSupplier, Supplier<Double> ySupplier,
        Supplier<Boolean> uncappedSupplier, Supplier<Boolean> fineControlSupplier,
        double reasonableMaxSpeed, boolean allowMovement
    ) {
        setName("AutoShootCommand");

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.uncappedSupplier = uncappedSupplier;
        this.fineControlSupplier = fineControlSupplier;
        this.reasonableMaxSpeed = reasonableMaxSpeed;
        
        this.allowMovement = allowMovement;
        this.idealShotVector = new ShotVector();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(
            IntakeSubsystem.getInstance(),
            PivotSubsystem.getInstance(),
            ShooterSubsystem.getInstance(),
            CommandSwerveDrivetrain.getInstance()
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.endEarly = false;

        try {
            this.speakerTranslation3d = Positions.getSpeakerTarget();
        }
        catch (RuntimeException e) {
            System.err.println("Alliance is empty ; cannot target SPEAKER.");
            this.endEarly = true;
            return;
        }

        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.endEarly) return;

        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        double botHeading = botState.Pose.getRotation().getDegrees();
        ChassisSpeeds botSpeeds = botState.speeds;

        // Ignore rotation because it should be none for accurate shooting either way.
        ShotVector botVector = new ShotVector(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond, 0);
        this.idealShotVector = calculateInitialShotVector().minus(botVector);

        CommandSwerveDrivetrain.getInstance().setControl(
            getDrivingControl(Rotation2d.fromDegrees(this.idealShotVector.getYaw()))
        );

        // TODO Check that the angle can even physically make the shot
        
        PivotSubsystem.getInstance().motionMagicPosition(this.idealShotVector.getPitch());
        ShooterSubsystem.getInstance().motionMagicVelocity(this.idealShotVector.getNormRps());

        ShotVector actualShotVector = ShotVector.fromYawPitchVelocity(
            botHeading,
            PivotSubsystem.getInstance().getPosition(),
            ShooterSubsystem.getInstance().getVelocity()
        );

        if (this.idealShotVector.getDistance(actualShotVector) <= 0.1 // TODO : Test tolerance
            // PivotSubsystem.getInstance().withinTolerance(this.idealShotVector.getPitch())
            // && ShooterSubsystem.getInstance().withinTolerance(this.idealShotVector.getNormRps())
            // && Math.min(
            //     Math.abs(this.idealShotVector.getYaw() - botHeading),
            //     360 - Math.abs(this.idealShotVector.getYaw() - botHeading)
            // ) <= ShootingConstants.FACING_ANGLE_TOLERANCE
        ) {
            IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
        }

        if (!IntakeSubsystem.getInstance().frontLaserHasNote()) {
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setSpeed(0);
        ShooterSubsystem.getInstance().setSpeed(0);
        PivotSubsystem.getInstance().motionMagicPosition(PivotConstants.ABOVE_LIMELIGHT_ANGLE);
        
        this.timer.stop();
    }

    /**
     * Helper that finds the ideal ShotVector from the bot's current position.
     * @return The ShotVector.
     */
    private ShotVector calculateInitialShotVector() {
        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        Pose2d botPose = botState.Pose;

        double distance = botPose.getTranslation().getDistance(this.speakerTranslation3d.toTranslation2d());
        double yaw = Units.radiansToDegrees(Math.atan2(
            this.speakerTranslation3d.getY() - botPose.getY(),
            this.speakerTranslation3d.getX() - botPose.getX()
        ) + Math.PI);
        double velocity = ShootingConstants.CALCULATE_SHOOTER_VELOCITY.apply(distance);
        double pitch = ShootingConstants.CALCULATE_SHOOTER_PITCH.apply(distance, velocity);

        return ShotVector.fromYawPitchVelocity(yaw, pitch, velocity);
    }

    /**
     * Helper that gets the control for driving while shooting.
     * @param targetRotation - The direction to face as a Rotation2d.
     * @return The SwerveRequest.
     */
    private SwerveRequest getDrivingControl(Rotation2d targetRotation) {
        if (!this.allowMovement) {
            return fieldCentricFacingAngle_withDeadband
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetRotation);
        }

        final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;

        boolean topSpeed = this.uncappedSupplier.get();
        boolean fineControl = this.fineControlSupplier.get();

        double velocityX = this.xSupplier.get()
            * (topSpeed ? MaxSpeed : this.reasonableMaxSpeed)
            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);
        double velocityY = this.ySupplier.get()
            * (topSpeed ? MaxSpeed : this.reasonableMaxSpeed)
            * (fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1);
        
        return fieldCentricFacingAngle_withDeadband
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withTargetDirection(targetRotation);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.endEarly || this.timer.hasElapsed(0.25);
    }
}
