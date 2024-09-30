// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that uses the operator's triggers to move the pivot. */
public class ManuallyPivotCommand extends Command {
    private final Supplier<Double> negativeSpeed;
    private final Supplier<Double> positiveSpeed;

    private final SlewRateLimiter speedLimiter;

    private boolean holding;

    /**
     * Creates a new ExampleCommand.
     * @param positiveSpeed - Expects a value between 0 and 1.0.
     * @param negativeSpeed - Expects a value between 0 and 1.0.
     */
    public ManuallyPivotCommand(Supplier<Double> positiveSpeed, Supplier<Double> negativeSpeed) {
        setName("ExampleCommand");

        this.positiveSpeed = positiveSpeed;
        this.negativeSpeed = negativeSpeed;
        this.holding = false;
        this.speedLimiter = new SlewRateLimiter(0.3);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(PivotSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.speedLimiter.reset(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double positive = positiveSpeed.get();
        positive = positive <= 0.02 ? 0 : positive;
        double negative = negativeSpeed.get();
        negative = negative <= 0.02 ? 0 : negative;
        
        // If no input, hold position
        if (!this.holding && positive == 0 && negative == 0) {
            PivotSubsystem.getInstance().motionMagicPosition(PivotSubsystem.getInstance().getPosition());
            this.speedLimiter.reset(0);
            this.holding = true;
        }
        // If press both input, let pivot fall
        else if (positive != 0 && negative != 0) {
            this.speedLimiter.reset(0);
            PivotSubsystem.getInstance().setPivotSpeed(0);
            this.holding = false;
        }
        else if (!(positive == 0 && negative == 0)) {
            PivotSubsystem.getInstance().setPivotSpeed(
                this.speedLimiter.calculate(positive - negative) * 0.2,
                true
            );
            this.holding = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Never ends because it is a default command
    }
}
