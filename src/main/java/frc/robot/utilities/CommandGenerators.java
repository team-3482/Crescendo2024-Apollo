package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.DriveToNoteCommand;
import frc.robot.constants.Constants.ShootingConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake.RunIntakeCommand;
import frc.robot.pivot.PivotCommand;
import frc.robot.shooter.ShootCommand;
import frc.robot.shooter.ShooterSubsystem;

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
                CommandGenerators.ResetPivotToIdlePositionCommand(),
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
     * A command that drives to a note and intakes it.
     * @return The command.
     */
    public static Command AutonIntakeNote() {
        return Commands.race(
            new DriveToNoteCommand().andThen(Commands.waitSeconds(2)),
            CommandGenerators.IntakeCommand()
        );
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
    public static Command ResetPivotToIdlePositionCommand() {
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
            new ShootCommand(ShootingConstants.MIN_POSITION_VELOCITY[1])
        );
    }

    /**
     * A command that will pivot and shoot for {@link ShootingConstants#PIVOT_POSITION_AMP}.
     * @return The command.
     */
    public static Command ShootAmpUpCloseCommand() {
        return Commands.sequence(
            new PivotCommand(ShootingConstants.PIVOT_POSITION_AMP),
            new ShootCommand(ShootingConstants.SHOOTER_SPEED_AMP)
        );
    }
}