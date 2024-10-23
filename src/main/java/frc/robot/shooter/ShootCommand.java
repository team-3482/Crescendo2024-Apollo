// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.intake.IntakeSubsystem;

/** Shoots a Note. */
public class ShootCommand extends Command {
    private double velocity;
    
    private boolean atVelocity;
    private boolean hitsLimelight;
    
    private Timer timer;

    /**
     * Creates a new ShootCommand.
     * @param velocity - The speed at which the rollers should run in rot/s.
     * @apiNote This value is clamped by {@link ShooterConstants#CRUISE_SPEED}.
     * Will not shoot if the shooter is below {@link PivotConstants#ABOVE_LIMELIGHT_ANGLE}.
     */
    public ShootCommand(double velocity) {
        setName("ShootCommand");
        
        this.velocity = velocity;
        this.timer = new Timer();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(IntakeSubsystem.getInstance(), ShooterSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.atVelocity = false;
        this.timer.reset();
        this.hitsLimelight = !ShooterSubsystem.getInstance().motionMagicVelocity(this.velocity);

        if (this.hitsLimelight) {
            System.out.println("ShootCommand | Current pivot position hits limelight.");
            return;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (this.hitsLimelight) return;
        
        if (!this.atVelocity && ShooterSubsystem.getInstance().withinTolerance(this.velocity)) {
            this.atVelocity = true;
        }

        if (this.atVelocity) {
            IntakeSubsystem.getInstance().motionMagicVelocity(IntakeConstants.IDEAL_INTAKE_VELOCITY);
        }

        if (!IntakeSubsystem.getInstance().frontLaserHasNote()) {
            this.timer.start();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setSpeed(0);
        IntakeSubsystem.getInstance().setSpeed(0);

        this.timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.hitsLimelight || this.timer.hasElapsed(0.25);
    }
}
