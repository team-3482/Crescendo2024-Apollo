// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;

/** Shoots a Note. */
public class PivotCommand extends Command {
    private final double position;
    private boolean end;

    /**
     * Creates a new PivotCommand.
     * @param position - The position to pivot to. Used as a fallback if position cannot be calculated.
     * @param end - Whether to end the Command when at the position.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public PivotCommand(double position, boolean end) {
        setName("PivotCommand");
        
        this.position = position;
        this.end = end;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(PivotSubsystem.getInstance());
    }

    /**
     * Creates a new PivotCommand.
     * @param position - The position to pivot to.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public PivotCommand(double position) {
        this(position, true);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        PivotSubsystem.getInstance().motionMagicPosition(this.position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.end && PivotSubsystem.getInstance().withinTolerance(this.position);
    }
}
