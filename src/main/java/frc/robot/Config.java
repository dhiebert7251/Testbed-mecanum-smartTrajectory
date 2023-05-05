// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/** Add your docs here. */
public class Config {
    
    //set scale value for max speed
    public static GenericEntry kDriveSpeedScale =
        Shuffleboard.getTab("ConfigTab")
            .add("Speed Scale", 1)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withPosition(1, 1)
            .withSize(2, 1)
            .getEntry();
}
