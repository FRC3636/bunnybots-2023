package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.CANDevice;

public final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.6);
    // Distance between front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(24.6);

    public static final Translation2d[] MODULE_POSITIONS = new Translation2d[]{
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    };
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(MODULE_POSITIONS);

    public static final Rotation2d[] MODULE_ROTATIONS = new Rotation2d[]{
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(90)
    };

    // SPARK MAX CAN IDs
    public static final int FRONT_LEFT_DRIVING_CAN_ID = CANDevice.FrontLeftDrivingMotor.getId();
    public static final int REAR_LEFT_DRIVING_CAN_ID = CANDevice.BackLeftTurningMotor.getId();
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = CANDevice.FrontRightDrivingMotor.getId();
    public static final int REAR_RIGHT_DRIVING_CAN_ID = CANDevice.BackRightTurningMotor.getId();

    public static final int FRONT_LEFT_TURNING_CAN_ID = CANDevice.FrontLeftTurningMotor.getId();
    public static final int REAR_LEFT_TURNING_CAN_ID = CANDevice.BackLeftTurningMotor.getId();
    public static final int FRONT_RIGHT_TURNING_CAN_ID = CANDevice.FrontRightTurningMotor.getId();
    public static final int REAR_RIGHT_TURNING_CAN_ID = CANDevice.BackRightTurningMotor.getId();

    public static final Rotation3d GYRO_ROTATION = new Rotation3d(0, 0, -Math.PI / 2);

    public static final double GYRO_ADJUSTMENT_RATE = 0.017;
}
