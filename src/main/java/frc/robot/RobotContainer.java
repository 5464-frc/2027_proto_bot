// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.SmartLogger;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
    instantiateSubsystems(new Subsystem[]{
      new SmartLogger()
    });
  }

  private void instantiateSubsystems(Subsystem[] passthroughSubsystems){
    for (Subsystem currentSubsystem : passthroughSubsystems) {
      CommandScheduler.getInstance().registerSubsystem(currentSubsystem);
    }
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
