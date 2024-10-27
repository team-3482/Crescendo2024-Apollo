// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ShooterConstants;
import frc.robot.constants.PhysicalConstants.ShooterConstants.ShooterSlot0Gains;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile ShooterSubsystem instance;
    private static Object mutex = new Object();

    public static ShooterSubsystem getInstance() {
        ShooterSubsystem result = instance;

        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new ShooterSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX topShooterMotor = new TalonFX(ShooterConstants.TOP_SHOOTER_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("ShooterSubsystem", BuiltInLayouts.kGrid)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 2, "Label position", "TOP"))
        .withSize(4, 3)
        .withPosition(6, 4);
    private GenericEntry shuffleboardVelocityBar_TopMotor = shuffleboardLayout
        .add("Top Velocity (rps)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", ShooterConstants.CRUISE_SPEED + 10, "Num tick marks", 6))
        .withSize(4, 2)
        .withPosition(0, 0)
        .getEntry();
    private GenericEntry shuffleboardVelocityBar_BottomMotor = shuffleboardLayout
        .add("Bottom Velocity (rps)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", ShooterConstants.CRUISE_SPEED + 10, "Num tick marks", 6))
        .withSize(4, 2)
        .withPosition(0, 1)
        .getEntry();

    /** Creates a new ShooterSubsystem. */
    private ShooterSubsystem() {
        super("ShooterSubsystem");

        configureMotors();

        // 20 ms update frequency (1 robot cycle)
        this.topShooterMotor.getVelocity().setUpdateFrequency(50);
        this.bottomShooterMotor.getVelocity().setUpdateFrequency(50);

        this.topShooterMotor.setControl(new Follower(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, true));
    }
    
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        this.shuffleboardVelocityBar_TopMotor.setDouble(
            this.topShooterMotor.getVelocity().getValueAsDouble());
        this.shuffleboardVelocityBar_BottomMotor.setDouble(
            this.bottomShooterMotor.getVelocity().getValueAsDouble());
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = ShooterSlot0Gains.kG;
        slot0Configs.kS = ShooterSlot0Gains.kS;
        slot0Configs.kV = ShooterSlot0Gains.kV;
        slot0Configs.kA = ShooterSlot0Gains.kA;
        slot0Configs.kP = ShooterSlot0Gains.kP;
        slot0Configs.kI = ShooterSlot0Gains.kI;
        slot0Configs.kD = ShooterSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.ACCELERATION;
        
        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 60;
        currentLimitsConfigs.SupplyCurrentThreshold = 60;
        currentLimitsConfigs.SupplyTimeThreshold = 0;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        this.bottomShooterMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        this.topShooterMotor.getConfigurator().apply(configuration);
    }

    /**
     * Aims for a velocity using MotionMagicVelocity.
     * @param velocity - In rotations/sec.
     * @return Whether set the velocity successfully (true) or aborted due to pivot angle (false).
     * @apiNote This value is clamped by {@link ShooterConstants#CRUISE_SPEED}.
     * That is the maximum speed of the rollers.
     * <li>Shooting is disabled if the pivot is below the {@link PivotConstants#ABOVE_LIMELIGHT_ANGLE}.
     */
    public boolean motionMagicVelocity(double velocity) {
        if (PivotSubsystem.getInstance().getPosition() < PivotConstants.ABOVE_LIMELIGHT_ANGLE
            && velocity > 0
        ) {
            return false;
        }

        velocity = MathUtil.clamp(velocity, -ShooterConstants.CRUISE_SPEED, ShooterConstants.CRUISE_SPEED);

        MotionMagicVelocityVoltage control = motionMagicVelocityVoltage
            .withSlot(0)
            .withVelocity(velocity);
        
        this.bottomShooterMotor.setControl(control);
        // this.topShooterMotor.setControl(control);

        return true;
    }

    /**
     * Sets the motors to a speed.
     * @param speed - Between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        this.bottomShooterMotor.set(speed);
        // this.topShooterMotor.set(speed);
    }

    /**
     * Gets the mechanism velocities of the two motors, averaged.
     * @return The velocity in rot/s.
     */
    public double getVelocity() {
        return this.bottomShooterMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Checks if the current velocity is within
     * {@link ShooterConstants#VELOCITY_TOLERANCE} of an input velocity.
     * @param velocity - The velocity to compare to in rot/s.
     */
    public boolean withinTolerance(double velocity) {
        return Math.abs(getVelocity() - velocity) <= ShooterConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Converts from rot/s to m/s for the shooter rollers.
     * @param speed - The speed in rot/s.
     * @return The speed in m/s.
     */
    public static double rotationsPerSecondToMetersPerSecond(double speed) {
        return speed * (2 * Math.PI * ShooterConstants.ROLLER_RADIUS_METERS);
    }

    /**
     * Converts from m/s to rot/s for the shooter rollers.
     * @param speed - The speed in m/s.
     * @return The speed in rot/s.
     */
    public static double metersPerSecondToRotationsPerSecond(double speed) {
        return speed / (2 * Math.PI * ShooterConstants.ROLLER_RADIUS_METERS);
    }
}
