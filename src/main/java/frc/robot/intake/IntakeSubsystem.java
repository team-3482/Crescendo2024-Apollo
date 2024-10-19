// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Map;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ShuffleboardTabNames;
import frc.robot.constants.PhysicalConstants.IntakeConstants;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.PhysicalConstants.IntakeConstants.IntakeSlot0Gains;

public class IntakeSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile IntakeSubsystem instance;
    private static Object mutex = new Object();

    public static IntakeSubsystem getInstance() {
        IntakeSubsystem result = instance;

        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new IntakeSubsystem();
                }
            }
        }
        return instance;
    }

    private TalonFX leftIntakeMotor = new TalonFX(IntakeConstants.LEFT_INTAKE_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private TalonFX rightIntakeMotor = new TalonFX(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, RobotConstants.CTRE_CAN_BUS);
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    
    private DigitalInput backLaser = new DigitalInput(IntakeConstants.BACK_LASER_CHANNEL);
    private DigitalInput frontLaser = new DigitalInput(IntakeConstants.FRONT_LASER_CHANNEL);

    private final ShuffleboardLayout shuffleboardLayout = Shuffleboard.getTab(ShuffleboardTabNames.DEFAULT)
        .getLayout("IntakeSubsystem", BuiltInLayouts.kGrid)
        .withSize(2, 4)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 1, "Label position", "TOP"));
    private GenericEntry shuffleboardVelocityBar = shuffleboardLayout
        .add("Intake Velocity (rps)", 0)
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("Min", 0, "Max", 50, "Num tick marks", 5))
        .withSize(2, 2)
        .withPosition(0, 1)
        .getEntry();
    private GenericEntry shuffleboardLaserBoolean = shuffleboardLayout
        .add("Note", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", "black", "colorWhenTrue", "#ff7f00"))
        .withPosition(0, 0)
        .withSize(2, 2)
        .getEntry();

    /** Creates a new IntakeSubsystem. */
    private IntakeSubsystem() {
        super("IntakeSubsystem");

        configureMotionMagic();

        // 20 ms update frequency (1 robot cycle)
        this.rightIntakeMotor.getVelocity().setUpdateFrequency(50);

        this.leftIntakeMotor.setControl(new Follower(IntakeConstants.RIGHT_INTAKE_MOTOR_ID, true));
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        this.shuffleboardVelocityBar.setDouble(getVelocity());
        this.shuffleboardLaserBoolean.setBoolean(hasNote());

        // TODO ELEC : Fix lasers
        // System.out.print("back : " + backLaserHasNote());
        // System.out.println(" ; front : " + frontLaserHasNote());
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotionMagic() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = IntakeConstants.ROTOR_TO_MECHANISM_RATIO; 

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake; // Holds notes

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = IntakeSlot0Gains.kG;
        slot0Configs.kS = IntakeSlot0Gains.kS;
        slot0Configs.kV = IntakeSlot0Gains.kV;
        slot0Configs.kA = IntakeSlot0Gains.kA;
        slot0Configs.kP = IntakeSlot0Gains.kP;
        slot0Configs.kI = IntakeSlot0Gains.kI;
        slot0Configs.kD = IntakeSlot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.ACCELERATION;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Bottom motor inverted.
        this.rightIntakeMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Top motor not inverted.
        this.leftIntakeMotor.getConfigurator().apply(configuration);
    }

    /**
     * Aims for a velocity using MotionMagicVelocity.
     * @param velocity - In rotations/sec.
     * @apiNote This value is clamped by {@link IntakeConstants#CRUISE_SPEED}.
     * That is the maximum speed of the subsystem.
     */
    public void motionMagicVelocity(double velocity) {
        velocity = MathUtil.clamp(velocity, -IntakeConstants.CRUISE_SPEED, IntakeConstants.CRUISE_SPEED);
        
        MotionMagicVelocityVoltage control = motionMagicVelocityVoltage
            .withSlot(0)
            .withVelocity(velocity);
        
        this.rightIntakeMotor.setControl(control);
    }

    /**
     * Set the speed of the intake motors.
     * @param speed - Between -1.0 and 1.0.
     */
    public void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        this.rightIntakeMotor.set(speed);
    }

    /**
     * Gets the mechanism velocity of the right motor.
     * @return The velocity in rot/s.
     */
    public double getVelocity() {
        return this.rightIntakeMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Checks if the current velocity is within
     * {@link IntakeConstants#VELOCITY_TOLERANCE} of an input velocity.
     * @param velocity - The velocity to compare to in rot/s.
     */
    public boolean withinTolerance(double velocity) {
        return Math.abs(getVelocity() - velocity) <= IntakeConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Gets if the back laser is broken.
     * @return If the laser is broken.
     */
    public boolean backLaserHasNote() {
        return !this.backLaser.get();
    }

    /**
     * Gets if the front laser is broken.
     * @return If the laser is broken.
     */
    public boolean frontLaserHasNote() {
        return !this.frontLaser.get();
    }

    /**
     * Gets whether the lasers are broken.
     * @return If the lasers are broken [back, front].
     */
    public boolean[] hasNotes() {
        return new boolean[]{ backLaserHasNote(), frontLaserHasNote() };
    }

    /** 
     * Checks whether there is a note in the intake.
     * @return Whether either laser is broken.
     */
    public boolean hasNote() {
        return backLaserHasNote() || frontLaserHasNote();
    }
}
