package frc.robot.subsystems.drivetrain

import MAXSwerveModuleIO
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.RobotContainer
import frc.robot.utils.PerCorner
import frc.robot.utils.cornerStatesToChassisSpeeds
import frc.robot.utils.toCornerSwerveModuleStates
import SimSwerveModuleIO
import org.littletonrobotics.junction.Logger
import edu.wpi.first.math.MathShared
import edu.wpi.first.math.util.Units
import ModuleIO
import ModuleIO.Inputs

// A singleton object representing the drivetrain.
object Drivetrain : Subsystem {





    private val modules = if (RobotBase.isReal()) {
        MODULE_CAN_DEVICES.zip(MODULE_POSITIONS).map { (can, pose) ->
            MAXSwerveModuleIO(can.first, can.second, pose.rotation)
        }.zip ( PerCorner.generate { Inputs() } )
    } else {
        PerCorner.generate { SimSwerveModuleIO() }.zip( PerCorner.generate { Inputs() } )
    }



    private val positions = MODULE_POSITIONS

    private val gyro: GyroIO = if (RobotBase.isReal()) {
        NavXGyroIO()
    } else {
        SimGyroIO()
    }

    val gyroInputs = GyroInputs()

    // Create swerve drivetrain kinematics using the translation parts of the module positions.
    private val kinematics = SwerveDriveKinematics(*positions.map(Pose2d::getTranslation).toList().toTypedArray())

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics, // swerve drive kinematics
        gyroInputs.rotation.toRotation2d(), // initial gyro rotation
        modules.map { it.second.position }.toTypedArray(), // initial module positions
        Pose2d() // initial pose
        // TODO: add odometry standard deviation
    )

    init {
        CommandScheduler.getInstance().registerSubsystem(this)
    }

    override fun periodic() {
        gyro.updateInputs(gyroInputs)
        // Update each of the modules.
        modules.forEach { (module, input) -> module.updateInputs(input) }
        // Update the estimated position.
        // poseEstimator.update(
        //     gyroInputs.rotation.toRotation2d(), modules.map { it.second.position }.toTypedArray()
        // )


        Logger.getInstance().recordOutput("Odometry/pose", estimatedPose)
        Logger.getInstance().recordOutput("Drive/ModulesState/Measured", *modules.map{it.second.state}.toTypedArray())
        Logger.getInstance().recordOutput("Drive/ModulesState/Desired", *modules.map{it.second.desiredState}.toTypedArray())



        // Log the swerve module states to SmartDashboard.
        SmartDashboard.putNumberArray(
            "Drive/ModuleStates/Measured",
            modules.map {it.second.state}.flatMap { listOf(it.angle.radians, it.speedMetersPerSecond) }
                .toTypedArray()
        )
        SmartDashboard.putNumberArray(
            "Drive/ModuleStates/Desired",
            modules.map {it.second.desiredState}.flatMap { listOf(it.angle.radians, it.speedMetersPerSecond) }
                .toTypedArray()
        )

        // Update the robot position on the SmartDashboard field.
        RobotContainer.field.robotPose = poseEstimator.estimatedPosition
    }

    // The current speed of chassis relative to the ground, assuming that the wheels have traction with the ground.
    val chassisSpeeds: ChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(modules.map {it.second.state})

    // Set the drivetrain to an X-formation to passively prevent movement in any direction.
    fun brake() {
        modules.zip(MODULE_POSITIONS).forEach { (module, position) ->
            // set the modules to radiate outwards from the chassis origin
            module.second.desiredState = SwerveModuleState(0.0, position.rotation)
        }
    }

    // Set the drivetrain to move with the given chassis speeds.
    //
    // Note that the speeds are relative to the chassis, not the field.
    fun drive(speeds: ChassisSpeeds) {
        
        // TODO: desaturate wheel speeds
        // SwerveDriveKinematics.desaturateWheelSpeeds(speeds, MAX_SPEED)

        kinematics.toCornerSwerveModuleStates(speeds).zip(modules).forEach { (state, module) ->
            module.first.setDesiredState(state) 
        }
    }

    fun zeroGyro() {
        gyro.reset()
    }

    // Get the estimated pose of the drivetrain using the pose estimator.
    val estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition


}

// Constants
internal val WHEEL_BASE_HALF: Double = Units.inchesToMeters(13.0)

internal val TRACK_WIDTH_HALF: Double = Units.inchesToMeters(14.0)

// Constants
internal val MODULE_POSITIONS = PerCorner(
    frontLeft = Pose2d(Translation2d(WHEEL_BASE_HALF, TRACK_WIDTH_HALF), Rotation2d.fromDegrees(-90.0)), 
    frontRight = Pose2d(Translation2d(WHEEL_BASE_HALF, -TRACK_WIDTH_HALF), Rotation2d.fromDegrees(180.0)), 
    backRight = Pose2d(Translation2d(-WHEEL_BASE_HALF, TRACK_WIDTH_HALF), Rotation2d.fromDegrees(0.0)), 
    backLeft = Pose2d(Translation2d(-WHEEL_BASE_HALF, -TRACK_WIDTH_HALF), Rotation2d.fromDegrees(90.0))
)


internal val MODULE_CAN_DEVICES = PerCorner(
    frontLeft = Pair(CANDevice.FrontLeftDrivingMotor, CANDevice.FrontLeftTurningMotor),
    frontRight = Pair(CANDevice.FrontRightDrivingMotor, CANDevice.FrontRightTurningMotor),
    backRight = Pair(CANDevice.BackRightDrivingMotor, CANDevice.BackRightTurningMotor),
    backLeft = Pair(CANDevice.BackLeftDrivingMotor, CANDevice.BackLeftTurningMotor),
)

