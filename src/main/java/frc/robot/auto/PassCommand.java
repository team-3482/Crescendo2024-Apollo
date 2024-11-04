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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Positions;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;
import frc.robot.swerve.TunerConstants;

/** Passes a Note. */
public class PassCommand extends Command {
    // Driving while shooting
    private final Supplier<Double> xSupplier;
    private final Supplier<Double> ySupplier;
    private final Supplier<Double> rotSupplier;

    private Translation2d passTranslation2d;
    private boolean facingBlue;
    
    private final boolean rotate;
    private boolean noAlliance;
    private Timer timer;

    /**
     * Creates a new AutoShootCommand.
     * @param xSupplier - Supplier for x robot movement from -1.0 to 1.0.
     * @param ySupplier - Supplier for y robot movement from -1.0 to 1.0.
     * @param rotSupplier - Supplier for rotational robot movement from -1.0 to 1.0.
     * @param rotate - Whether or not to check face the passing position..
     */
    public PassCommand( // TODO : Test and fix
        Supplier<Double> xSupplier, Supplier<Double> ySupplier,
        Supplier<Double> rotSupplier, boolean rotate
    ) {
        setName("PassCommand");

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.rotate = rotate;

        this.timer = new Timer();

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
        this.noAlliance = false;

        try {
            this.passTranslation2d = Positions.getPassTarget();
            this.facingBlue = DriverStation.getAlliance().get() == Alliance.Blue;
        }
        catch (RuntimeException e) {
            System.err.println("Alliance is empty ; cannot target SPEAKER or set rotation addition.");
            this.noAlliance = true;
            return;
        }

        this.timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.noAlliance) return;

        SwerveDriveState botState = CommandSwerveDrivetrain.getInstance().getState();
        Translation2d botTranslation = botState.Pose.getTranslation();
        double botHeading = botState.Pose.getRotation().getDegrees();

        Rotation2d yaw = new Rotation2d(Math.atan2(
            this.passTranslation2d.getY() - botTranslation.getY(),
            this.passTranslation2d.getX() - botTranslation.getX()
        ) + (this.facingBlue ? Math.PI : 0));

        CommandSwerveDrivetrain.getInstance().setControl(
            getDrivingControl(yaw)
        );
        
        PivotSubsystem.getInstance().motionMagicPosition(ShootingConstants.PIVOT_POSITION_PASS);
        ShooterSubsystem.getInstance().motionMagicVelocity(ShootingConstants.SHOOTER_SPEED_PASS);

        boolean pivotShooterWithinTolerance = ShooterSubsystem.getInstance().withinTolerance(ShootingConstants.SHOOTER_SPEED_PASS)
            && PivotSubsystem.getInstance().withinTolerance(ShootingConstants.PIVOT_POSITION_PASS);

        if (
            this.rotate
            && Math.min(
                Math.abs(yaw.getDegrees() - botHeading + (this.facingBlue ? 0 : 180)),
                360 - Math.abs(yaw.getDegrees() - botHeading + (this.facingBlue ? 0 : 180))
            ) <= ShootingConstants.YAW_TOLERANCE_PASS
            && pivotShooterWithinTolerance
        ) {
            // IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
            IntakeSubsystem.getInstance().setVoltage(IntakeConstants.IDEAL_INTAKE_VOLTAGE);
        }
        else if (!this.rotate && pivotShooterWithinTolerance) {
            // IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
            IntakeSubsystem.getInstance().setVoltage(IntakeConstants.IDEAL_INTAKE_VOLTAGE);

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
        return this.noAlliance || this.timer.hasElapsed(0.25);
    }

    /**
     * Helper that gets the control for driving while shooting.
     * @param targetRotation - The direction to face as a Rotation2d.
     * @return The SwerveRequest.
     */
    private SwerveRequest getDrivingControl(Rotation2d targetRotation) {
        final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle_withDeadband = new CommandSwerveDrivetrain.FieldCentricFacingAngle_PID_Workaround()
            .withDeadband(TunerConstants.reasonableMaxSpeed * ControllerConstants.DEADBAND)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (this.rotate) {
            return fieldCentricFacingAngle_withDeadband
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(targetRotation);
        }

        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.reasonableMaxSpeed * ControllerConstants.DEADBAND)
            .withRotationalDeadband(TunerConstants.reasonableMaxAngularRate * ControllerConstants.DEADBAND) // Add a deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
        double velocityX = this.xSupplier.get() * TunerConstants.reasonableMaxSpeed;
        double velocityY = this.ySupplier.get() * TunerConstants.reasonableMaxSpeed;
        
        return fieldCentricDrive_withDeadband
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(this.rotSupplier.get() * TunerConstants.reasonableMaxAngularRate);
    }
}
