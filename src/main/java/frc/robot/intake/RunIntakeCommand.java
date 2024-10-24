// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;

/** Runs the intake for a Note. */
public class RunIntakeCommand extends Command {
    /** Creates a new RunIntakeCommand. */
    public RunIntakeCommand() {
        setName("RunIntakeCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!IntakeSubsystem.getInstance().frontLaserHasNote()) {
            IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (IntakeSubsystem.getInstance().backLaserHasNote()) {
            IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.SLOW_INTAKE_VELOCITY);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return IntakeSubsystem.getInstance().frontLaserHasNote();
    }
}
