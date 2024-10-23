// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.templates;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
    // Thread-safe singleton design pattern.
    private static volatile ExampleSubsystem instance;
    private static Object mutex = new Object();

    public static ExampleSubsystem getInstance() {
        ExampleSubsystem result = instance;
        
        if (result == null) {
            synchronized (mutex) {
                result = instance;
                if (result == null) {
                    instance = result = new ExampleSubsystem();
                }
            }
        }
        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    private ExampleSubsystem() {
        super("ExampleSubsystem");
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {}
}
