// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class RobotContainer {
  public static final ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve Tab");

  public static final Drivetrain drivetrain = new Drivetrain();
  public static final BallIntake ballintake = new BallIntake();
  public static final BunnyIntake bunnyintake = new BunnyIntake();
  public static final Shooter shooter = new Shooter();
  public static final Indexer indexer = new Indexer();
  public static final Turret turret = new Turret();



  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
