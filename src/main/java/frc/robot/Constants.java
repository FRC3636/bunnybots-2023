package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class DriveConstants{



        public static final double MAX_SPEED_METERS_PER_SECOND = 0;
        //CAN IDs
        public static final int FRONT_LEFT_DRIVE_ID = 0;
        public static final int FRONT_LEFT_TURN_ID = 0;

        
        public static final int FRONT_RIGHT_DRIVE_ID = 0;
        public static final int FRONT_RIGHT_TURN_ID = 0;

        
        public static final int REAR_LEFT_DRIVE_ID = 0;
        public static final int REAR_LEFT_TURN_ID = 0;

      
        public static final int REAR_RIGHT_DRIVE_ID = 0;
        public static final int REAR_RIGHT_TURN_ID = 0;



        public static final boolean GYRO_REVERSED = false;

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


        public static final double GYRO_ADJUSTMENT_RATE = 0.0;
        public static final Rotation3d GYRO_ROTATION = new Rotation3d(0, 0, -3.1415926/2);

        public static final Rotation2d[] MODULE_ROTATIONS = new Rotation2d[]{
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(0),
            Rotation2d.fromDegrees(180),
            Rotation2d.fromDegrees(90)
        };

    }

    public static final class ModuleConstants {
        //TODO tune all this shit


        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean TURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60;
        public static final double WHEEL_DIAMETER_METERS = 0;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double DRIVING_MOTOR_REDUCTION = 0; 
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
                / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION; // meters
        public static final double DRIVING_ENCODER_VELOCITY_FACTOR = (WHEEL_CIRCUMFERENCE_METERS
                / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
        public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        public static final double DRIVING_P = 0.04;
        public static final double DRIVING_I = 0;
        public static final double DRIVING_D = 0;
        public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double DRIVING_MIN_OUTPUT = -1;
        public static final double DRIVING_MAX_OUTPUT = 1;

        public static final double TURNING_P = 2;
        public static final double TURNING_I = 0;
        public static final double TURNING_D = 0;
        public static final double TURNING_FF = 0;
        public static final double TURNING_MIN_OUTPUT = -1;
        public static final double TURNING_MAX_OUTPUT = 1;

        public static final CANSparkMax.IdleMode DRIVING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;
        public static final CANSparkMax.IdleMode TURNING_MOTOR_IDLE_MODE = CANSparkMax.IdleMode.kBrake;

        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50; // amps
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20; // amps
    }

    public static final class NeoMotorConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }

    public static final class BunnyIntake{
        public static final int HINGE_ID = 0;
        public static final int ROLLER_ID = 0;

    }

    public static final class BallIntake{
        public static final int HINGE_ID = 0;
        public static final int ROLLER_ID = 0;

        public static final int UP_LIMIT_SWITCH_CHANNEL = 0;
        public static final int DOWN_LIMIT_SWITCH_CHANNEL = 0;

        public static final double HINGE_SPEED = 0.3;

        public static final double INTAKE_DOWN_ROTATION = 0;

    }


    public static final class Turret{
        public static final int ID = 0;
    }

    public static final class Shooter{
        public static final int NEO_ID = 0;
        public static final int TALON_ID = 0;
    }

    public static final class Indexer{
        public static final int ID = 0;
    }


}
