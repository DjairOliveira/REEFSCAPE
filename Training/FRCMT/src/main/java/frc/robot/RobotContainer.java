// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevador;

public class RobotContainer {

  public final static CommandXboxController driverXbox = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driverXbox.a().onTrue(new Elevador(100, 0.05, 0.6));
    driverXbox.x().onTrue(new Elevador(10, 0.02, 0.4));
    driverXbox.b().onTrue(new Elevador(1, 0.02, 0.2));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
