// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain extends SubsystemBase {

    // Create MAXSwerveModules
    private final SwerveModules swerveModules;
    // The gyro sensor
    private final Gyro gyro = new Gyro();
    /**
     * Creates a new DriveSubsystem.
     */
    public Drivetrain() {

        this.swerveModules = new SwerveModules(
                new MAXSwerveModule(
                        DriveConstants.FRONT_LEFT_DRIVE_ID,
                        DriveConstants.FRONT_LEFT_TURN_ID,
                        DriveConstants.MODULE_ROTATIONS[0]
                ),
                new MAXSwerveModule(
                        DriveConstants.FRONT_RIGHT_DRIVE_ID,
                        DriveConstants.FRONT_RIGHT_TURN_ID,
                        DriveConstants.MODULE_ROTATIONS[1]
                ),
                new MAXSwerveModule(
                        DriveConstants.REAR_LEFT_DRIVE_ID,
                        DriveConstants.REAR_LEFT_TURN_ID,
                        DriveConstants.MODULE_ROTATIONS[2]
                ),
                new MAXSwerveModule(
                        DriveConstants.REAR_RIGHT_DRIVE_ID,
                        DriveConstants.REAR_RIGHT_TURN_ID,
                        DriveConstants.MODULE_ROTATIONS[3]
                )
        );


        RobotContainer.swerveTab.addNumber("Front Left", swerveModules.frontLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Front Right", swerveModules.frontRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Back Left", swerveModules.rearLeft::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);
        RobotContainer.swerveTab.addNumber("Back Right", swerveModules.rearRight::getSwerveEncoderPosition).withWidget(BuiltInWidgets.kGraph);

        RobotContainer.swerveTab.addNumber("Gyro", () -> gyro.getAngle().getRadians()).withWidget(BuiltInWidgets.kGraph);
    }

    @Override
    public void periodic() { 

        //TODO odometry :0

        // RobotContainer.poseEstimation.updateOdometry(
        //         getRotation(),
        //         getModulePositions()
        // );

        gyro.update();
        swerveModules.update();

        logModuleStates("Swerve State", swerveModules.getStates().asArray());
        logModuleStates("Swerve Set State", new SwerveModuleState[]{
                swerveModules.frontLeft.getSetState(),
                swerveModules.frontRight.getSetState(),
                swerveModules.rearLeft.getSetState(),
                swerveModules.rearRight.getSetState()
        });
        SmartDashboard.putNumberArray("Swerve Module Distance", new double[]{
                swerveModules.frontLeft.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.frontRight.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearLeft.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS,
                swerveModules.rearRight.getPosition().distanceMeters / Constants.ModuleConstants.WHEEL_CIRCUMFERENCE_METERS});

        SmartDashboard.putNumberArray("Swerve Module Distance Revolutions", new double[]{
                swerveModules.frontLeft.getPosition().distanceMeters,
                swerveModules.frontRight.getPosition().distanceMeters,
                swerveModules.rearLeft.getPosition().distanceMeters,
                swerveModules.rearRight.getPosition().distanceMeters});

        SmartDashboard.putNumber("Pitch", gyro.getRotation3d().getY());
    }

    private void logModuleStates(String key, SwerveModuleState[] swerveModuleStates) {
        List<Double> dataList = new ArrayList<>();
        for (SwerveModuleState swerveModuleState : swerveModuleStates) {
            dataList.add(swerveModuleState.angle.getRadians());
            dataList.add(swerveModuleState.speedMetersPerSecond);
        }
        SmartDashboard.putNumberArray(key,
                dataList.stream().mapToDouble(Double::doubleValue).toArray());
    }

    public Rotation2d getRotation() {
        Rotation2d rot = gyro.getAngle();

        if (DriveConstants.GYRO_REVERSED) {
            rot = rot.unaryMinus();
        }

        return rot;
    }

    public Rotation3d getRotation3d() {
        return gyro.getRotation3d();
    }

    public SwerveModulePosition[] getModulePositions() {
        return swerveModules.getPositions().asArray();
    }

    public SwerveModuleState[] getModuleStates() {
        return swerveModules.getStates().asArray();
    }

    /**
     * Method to drive the drivetrain using chassis speeds.
     *
     * @param speeds The chassis speeds.
     */
    public void drive(ChassisSpeeds speeds) {
        double dt = Robot.kDefaultPeriod;
        Pose2d poseVel = new Pose2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));
        Twist2d twistVel = new Pose2d().log(poseVel);
        ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
                twistVel.dx / dt, twistVel.dy / dt, twistVel.dtheta / dt);

        SwerveModuleState[] swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(correctedSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        swerveModules.setDesiredStates(
                new SwerveModules.States(
                        // front left
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        // front right
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        // rear left
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        // rear right
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                ).asArray()
        );
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
        swerveModules.setDesiredStates(desiredStates);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        swerveModules.resetEncoders();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return gyro.getAngle();
    }

    public Translation2d getRelativeVelocity() {
        ChassisSpeeds chassisSpeeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());

        return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate().getDegrees();
    }

    public Boolean getNavXStatus() {
        if (RobotBase.isSimulation()) 
            return false;

        return gyro.isConnected();
    }

}
