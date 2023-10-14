// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // Derived from https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    LimelightHelpers.LimelightResults latestResults = LimelightHelpers.getLatestResults("limelight");
    if (latestResults.targetingResults.targets_Retro.length != 0) {
      double ty = LimelightHelpers.getTY("limelight");
      double angleToGoal = Units.degreesToRadians(Constants.Shooter.LIMELIGHT_ANGLE + ty);
      double distanceToBucket = (Constants.Shooter.BUCKET_HEIGHT - Constants.Shooter.LIMELIGHT_HEIGHT) / Math.tan(angleToGoal);
      System.out.println("Distance to bucket is " + distanceToBucket + " inches!");
    } else {
      System.out.println("No targets to measure :(");
    }
  }
}
