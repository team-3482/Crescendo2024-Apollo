// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.Constants.ShootingFunctions;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.limelights.VisionSubsystem;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.utilities.ShotVector;

/** Shoots a Note autonomously. */
public class AutoShootCommand extends Command {
    // Driving while shooting
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final double maxSpeed;

    private Translation3d speakerTranslation3d;
    private boolean facingBlue;
    
    private ShotVector idealShotVector;
    
    private final boolean allowMovement;
    private final boolean checkDistance;
    
    private boolean noSpeaker;
    private boolean withinDistance;
    private Timer timer;

    /**
     * Creates a new AutoShootCommand.
     * @param xSupplier - Supplier for x robot movement from -1.0 to 1.0.
     * @param ySupplier - Supplier for y robot movement from -1.0 to 1.0.
     * @param allowMovement - Whether to allow driver input while shooting.
     * @param checkDistance - Whether or not to check the distance or allow any distance shooting.
     */
    private AutoShootCommand(
        Supplier<Double> xSupplier, Supplier<Double> ySupplier,
        boolean allowMovement, boolean checkDistance
    ) {
        setName("AutoShootCommand");

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.maxSpeed = ShootingConstants.MAX_MOVEMENT_SPEED;
        
        this.allowMovement = allowMovement;
        this.checkDistance = checkDistance;

        this.idealShotVector = new ShotVector();
        this.timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(
            IntakeSubsystem.getInstance(),
            PivotSubsystem.getInstance(),
            ShooterSubsystem.getInstance(),
            CommandSwerveDrivetrain.getInstance()
        );
    }

    /**
     * Creates a new AutoShootCommand that does not allow movement.
     * @param checkDistance - Whether or not to check the distance or allow any distance shooting.
     */
    public AutoShootCommand(boolean checkDistance) {
        this(null, null, false, checkDistance);
    }

    /**
     * Creates a new AutoShootCommand that allows movement.
     * @param xSupplier - Supplier for x robot movement from -1.0 to 1.0.
     * @param ySupplier - Supplier for y robot movement from -1.0 to 1.0.
     * @apiNote Limits the distance for shooting to a valid distance.
     */
    public AutoShootCommand(Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
        this(xSupplier, ySupplier, true, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.noSpeaker = false;
        this.withinDistance = false;

        try {
            this.speakerTranslation3d = Positions.getSpeakerTarget();
            this.facingBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        }
        catch (RuntimeException e) {
            System.err.println("Alliance is empty ; cannot target SPEAKER or set rotation addition.");
            this.noSpeaker = true;
            return;
        }

        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.noSpeaker) return;

        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        ChassisSpeeds botSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(botState.speeds, botState.Pose.getRotation());
        double botHeading = botState.Pose.getRotation().getDegrees();

        // Ignore rotation because it should be none for accurate shooting either way.
        ShotVector botVector = new ShotVector(botSpeeds.vxMetersPerSecond, botSpeeds.vyMetersPerSecond, 0);
        this.idealShotVector = calculateInitialShotVector().plus(botVector);

        CommandSwerveDrivetrain.getInstance().setControl(
            getDrivingControl(Rotation2d.fromDegrees(this.idealShotVector.getYaw()))
        );
        
        if (!this.withinDistance) return;
            
        double distance = botState.Pose.getTranslation().getDistance(this.speakerTranslation3d.toTranslation2d());
        double adjustedPitch = this.idealShotVector.getAdjustedPitch(distance);
        
        PivotSubsystem.getInstance().motionMagicPosition(adjustedPitch);
        ShooterSubsystem.getInstance().motionMagicVelocity(this.idealShotVector.getNormRps());

        // System.out.printf(
        //     "Yaw %.2f <= %.2f ; Pitch %.2f <= %.2f ; Velocity %.2f <= %.2f%n",
        //     Math.min(
        //         Math.abs(this.idealShotVector.getYaw() - botHeading + (this.facingBlue ? 0 : 180)),
        //         360 - Math.abs(this.idealShotVector.getYaw() - botHeading + (this.facingBlue ? 0 : 180))
        //     ),
        //     ShootingFunctions.CALCULATE_YAW_TOLERANCE.apply(distance),
        //     Math.abs(PivotSubsystem.getInstance().getPosition() - adjustedPitch),
        //     ShootingFunctions.CALCULATE_PITCH_TOLERANCE.apply(distance),
        //     Math.abs(ShooterSubsystem.getInstance().getVelocity() - this.idealShotVector.getNormRps()),
        //     ShooterSubsystem.metersPerSecondToRotationsPerSecond(
        //         ShootingFunctions.CALCULATE_SPEED_TOLERANCE.apply(distance, this.idealShotVector.getPitch())
        //     )
        // );

        if (
            VisionSubsystem.getInstance().recentVisionData()
            && Math.min(
                Math.abs(this.idealShotVector.getYaw() - botHeading + (this.facingBlue ? 0 : 180)),
                360 - Math.abs(this.idealShotVector.getYaw() - botHeading + (this.facingBlue ? 0 : 180))
            ) <= ShootingFunctions.CALCULATE_YAW_TOLERANCE.apply(distance)
            && Math.abs(PivotSubsystem.getInstance().getPosition() - adjustedPitch)
                <= ShootingFunctions.CALCULATE_PITCH_TOLERANCE.apply(distance)
            && Math.abs(ShooterSubsystem.getInstance().getVelocity() - this.idealShotVector.getNormRps())
                <= ShooterSubsystem.metersPerSecondToRotationsPerSecond(
                    ShootingFunctions.CALCULATE_SPEED_TOLERANCE.apply(distance, this.idealShotVector.getPitch())
                )
        ) {
            // IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY / 2);
            IntakeSubsystem.getInstance().setVoltage(IntakeConstants.IDEAL_INTAKE_VOLTAGE / 2);
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
        
        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.noSpeaker || (!this.allowMovement && !this.withinDistance) || this.timer.hasElapsed(0.25);
    }

    /**
     * Helper that finds the ideal ShotVector from the bot's current position.
     * @return The ShotVector.
     */
    private ShotVector calculateInitialShotVector() {
        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        Translation2d botTranslation = botState.Pose.getTranslation();
        double distance = botTranslation.getDistance(this.speakerTranslation3d.toTranslation2d());

        double yaw = Units.radiansToDegrees(Math.atan2(
            this.speakerTranslation3d.getY() - botTranslation.getY(),
            this.speakerTranslation3d.getX() - botTranslation.getX()
        ) + (this.facingBlue ? Math.PI : 0));

        if (this.checkDistance && distance > ShootingConstants.MAX_SHOOTING_DISTANCE) {
            System.err.println(String.format(
                "AutoShootCommand | Too far from SPEAKER. (%.2f > %.2f)",
                distance, ShootingConstants.MAX_SHOOTING_DISTANCE
            ));
            this.withinDistance = false;
            return ShotVector.fromYawPitchVelocity(yaw, 1, 1);
        }
        else {
            this.withinDistance = true;
        }

        double velocity = ShooterSubsystem.rotationsPerSecondToMetersPerSecond(
            ShootingFunctions.CALCULATE_SHOOTER_VELOCITY.apply(distance)
        );
        double pitch = ShootingFunctions.CALCULATE_SHOOTER_PITCH.apply(distance, velocity);

        return ShotVector.fromYawPitchVelocity(yaw, pitch, velocity);
    }

    /**
     * Helper that gets the control for driving while shooting.
     * @param targetRotation - The direction to face as a Rotation2d.
     * @return The SwerveRequest.
     */
    private SwerveRequest getDrivingControl(Rotation2d targetRotation) {
        final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
            .withDeadband(this.maxSpeed * ControllerConstants.DEADBAND)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (!this.allowMovement) {
            return fieldCentricFacingAngle_withDeadband
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetRotation);
        }
        
        double velocityX = this.xSupplier.get() * this.maxSpeed;
        double velocityY = this.ySupplier.get() * this.maxSpeed;
        
        return fieldCentricFacingAngle_withDeadband
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withTargetDirection(targetRotation);
    }
}
