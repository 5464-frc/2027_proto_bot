// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems.SDS;
import frc.robot.Subsystems.SmartLogger;
import frc.robot.Subsystems.Vision;

public class RobotContainer {
  List<String> registerSubsystems = new ArrayList<>();
  public RobotContainer() {
    configureBindings();
    instantiateSubsystems(new Subsystem[] {
        new SmartLogger(),
        new SDS(),
        new Vision()
    });
  }

  private void instantiateSubsystems(Subsystem[] passthroughSubsystems) {
    for (Subsystem currentSubsystem : passthroughSubsystems) {
      System.out.println(currentSubsystem.getName());
      if (registerSubsystems.contains(currentSubsystem.getName())){
        //avoid repeat registration warnings
        continue;
      }
      registerSubsystems.add(currentSubsystem.getName());
      CommandScheduler.getInstance().registerSubsystem(currentSubsystem);
    }
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
