package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoShootCommand;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.RunIntakeCommand;
import frc.robot.limelights.VisionSubsystem;
import frc.robot.pivot.PivotCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.swerve.CommandSwerveDrivetrain;

/** A class that holds static methods that perform multiple functions. */
public final class CommandGenerators {
    // GENERAL
    //
    //
    //
    //
    //
    //
    /**
     * A command that cancels all running commands.
     * @return The command.
     */
    public static Command CancelAllCommands() {
        return Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll());
    }

    /**
     * A command that moves the pivot down and intakes.
     * @return The command.
     */
    public static Command IntakeCommand() {
        return Commands.sequence(
            CommandGenerators.PivotToIdlePositionCommand(),
            new RunIntakeCommand()
        );
    }

    // DRIVER
    //
    //
    //
    //
    //
    //
    /**
     * A command that takes the current orientation of the robot
     * and makes it X forward for field-relative maneuvers.
     * @return The command.
     */
    public static Command SetForwardDirectionCommand() {
        return Commands.runOnce(() -> CommandSwerveDrivetrain.getInstance().seedFieldRelative());
    }

    /**
     * A command that resets the odometry using data from the Limelights.
     * @return The command.
     * @apiNote If the Limelight returns an empty Pose2d, this command is a no-op.
     */
    public static Command ResetPoseUsingLimelightCommand() {
        return Commands.runOnce(() -> {
            Pose2d estimatedPose = VisionSubsystem.getInstance().getEstimatedPose();
            if (!estimatedPose.equals(new Pose2d())) {
                CommandSwerveDrivetrain.getInstance().seedFieldRelative(estimatedPose);
            }
        });
    }

    /**
     * A command that drives to a note and intakes it.
     * @return The command.
     */
    public static Command AutonIntakeNoteCommand() {
        return Commands.race(
            new DriveToNoteCommand().andThen(Commands.waitSeconds(2)),
            CommandGenerators.IntakeCommand()
        );
    }

    /**
     * A command that shoots while allowing robot movement.
     * @return The command.
     */
    public static Command AutoShootNoteMovementCommand() {
        CommandXboxController driverController = RobotContainer.getInstance().getDriverController();
        return new AutoShootCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX()
        );
    }

    /**
     * A command that shoots a note without allowing robot movement.
     * @return The command.
     */
    public static Command AutoShootNoteStaticCommand() {
        return new AutoShootCommand();
    }

    // OPERATOR
    //
    //
    //
    //
    //
    //
    /**
     * A command that moves the pivot to {@link PivotConstants#ABOVE_LIMELIGHT_ANGLE}.
     * @return The command.
     */
    public static Command PivotToIdlePositionCommand() {
        return new PivotCommand(PivotConstants.ABOVE_LIMELIGHT_ANGLE);
    }

    /**
     * A command that runs the intake manually.
     * @return The command.
     */
    public static Command ManualIntakeCommand() {
        return IntakeSubsystem.getInstance().runEnd(
            () -> IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY),
            () -> IntakeSubsystem.getInstance().setSpeed(0)
        );
    }

    /**
     * A command that runs the intake and shooter backwards manually.
     * @return The command.
     */
    public static Command ManuallyReverseIntakeCommand() {
        return Commands.runEnd(
            () -> {
                ShooterSubsystem.getInstance().motionMagicVelocity(-ShootingConstants.MIN_POSITION_VELOCITY[1]);
                IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_EJECT_VELOCITY);
            },
            () -> {
                ShooterSubsystem.getInstance().setSpeed(0);
                IntakeSubsystem.getInstance().setSpeed(0);
            },
            ShooterSubsystem.getInstance(), IntakeSubsystem.getInstance()
        );
    }

    /**
     * A command that will pivot and shoot for {@link ShootingConstants#PIVOT_POSITION_SPEAKER}.
     * @return The command.
     */
    public static Command ShootSpeakerUpCloseCommand() {
        return Commands.sequence(
            new PivotCommand(ShootingConstants.PIVOT_POSITION_SPEAKER),
            new ShootCommand(ShootingConstants.MIN_POSITION_VELOCITY[1], 0.3)
        );
    }

    /**
     * A command that will pivot and shoot for {@link ShootingConstants#PIVOT_POSITION_AMP}.
     * @return The command.
     */
    public static Command ShootAmpUpCloseCommand() {
        return Commands.sequence(
            new PivotCommand(ShootingConstants.PIVOT_POSITION_AMP),
            new ShootCommand(ShootingConstants.SHOOTER_SPEED_AMP, 0.5)
        );
    }
}