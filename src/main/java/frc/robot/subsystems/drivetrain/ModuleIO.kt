import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.robot.CANDevice
import frc.robot.Robot
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController
import frc.robot.utils.TAU
import frc.robot.utils.constants
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs


interface ModuleIO {
    class ModuleRawInputs: LoggableInputs {

        // The current "state" of the swerve module.
        //
        // This is essentially the velocity of the wheel,
        // and includes both the speed and the angle
        // in which the module is currently traveling.
        // var state: SwerveModuleState

        var speed: Double = 0.0
        var rotation: Double = 0.0

        // The desired state of the module.
        //
        // This is the wheel velocity that we're trying to get to.
        var desiredSpeed: Double = 0.0
        var desiredRotation: Double = 0.0

        // The measured position of the module.
        //
        // This is a vector with direction equal to the current angle of the module,
        // and magnitude equal to the total signed distance traveled by the wheel.

        var direction: Double = 0.0
        var distance: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Current Speed(Meters/Sec)", speed)
            table?.put("Current Angle", rotation)
            table?.put("Target Speed (Meters / Sec", desiredSpeed)
            table?.put("Target Angle", desiredRotation)
            table?.put("Distance Travelled", distance)
            table?.put("Angle", direction)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Current Speed(Meters/Sec)", speed)?.let { speed = it}
            table?.getDouble("Current Angle", rotation)?.let { rotation = it}
            table?.getDouble("Target Speed (Meters / Sec", desiredSpeed)?.let { desiredSpeed = it}
            table?.getDouble("Target Angle", desiredRotation)?.let { desiredRotation = it }
            table?.getDouble("Distance Travelled", distance)?. let {desiredRotation = it}
            table?.getDouble("Angle", direction)?.let {direction = it}
        }

    }

    fun updateInputs(inputs: ModuleInputs)

    fun periodic(inputs: ModuleInputs){}
}

class ModuleInputs {
    private val rawInputs: ModuleIO.ModuleRawInputs = ModuleIO.ModuleRawInputs()

    var state: SwerveModuleState = SwerveModuleState()
    var desiredState: SwerveModuleState = SwerveModuleState()
    var position: SwerveModulePosition = SwerveModulePosition()

     fun updateRaw(){
        rawInputs.speed = state.speedMetersPerSecond
        rawInputs.rotation = state.angle.radians
        rawInputs.desiredSpeed = desiredState.speedMetersPerSecond
        rawInputs.desiredRotation = desiredState.angle.radians
        rawInputs.distance = position.distanceMeters
        rawInputs.direction = position.angle.radians
    }
}

class MAXSwerveModuleIO(drivingCAN: CANDevice, turningCAN: CANDevice, val chassisAngle: Rotation2d) : ModuleIO {

    override fun updateInputs(inputs: ModuleInputs){

        inputs.state = SwerveModuleState(
            drivingEncoder.velocity, Rotation2d.fromRadians(turningEncoder.position).plus(chassisAngle)
        )
        inputs.position = SwerveModulePosition(drivingEncoder.position, inputs.state.angle)

        val value = SwerveModuleState(0.0, chassisAngle.unaryMinus())
        // transform the state into module space
        val corrected = SwerveModuleState(value.speedMetersPerSecond, value.angle.minus(chassisAngle))
        // optimize the state to avoid rotating more than 90 degrees
        val optimized = SwerveModuleState.optimize(corrected, Rotation2d.fromRadians(turningEncoder.position))

        // set the pid controller setpoints
        drivingPIDController.setReference(optimized.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity)
        turningPIDController.setReference(optimized.angle.radians, CANSparkMax.ControlType.kPosition)

        // update the field using the original chassis space state
        inputs.desiredState = value
        inputs.updateRaw()
    }


    // SPARK MAXs for the driving and turning motors, respectively
    private val drivingSpark = CANSparkMax(drivingCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()

        idleMode = CANSparkMax.IdleMode.kBrake
        setSmartCurrentLimit(DRIVING_CURRENT_LIMIT)

        burnFlash()
    }

    private val turningSpark = CANSparkMax(turningCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()

        idleMode = CANSparkMax.IdleMode.kBrake
        setSmartCurrentLimit(TURNING_CURRENT_LIMIT)

        burnFlash()
    }


    // the driving motor encoder is relative because we care
    // only about how far the wheel travels, not where it started
    private val drivingEncoder = drivingSpark.encoder.apply {
        // convert native units of rotations and RPM to meters and meters per second
        positionConversionFactor = WHEEL_CIRCUMFERENCE * DRIVING_MOTOR_TO_WHEEL_GEARING
        velocityConversionFactor = WHEEL_CIRCUMFERENCE * DRIVING_MOTOR_TO_WHEEL_GEARING / 60.0
    }

    // whereas the turning encoder must be absolute so that
    // we know where the wheel is pointing
    private val turningEncoder = turningSpark.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).apply {
        // invert the encoder because the output shaft rotates opposite to the motor itself
        inverted = true

        // convert native units of rotations and RPM to radians and radians per second
        // tau = 2 * pi = circumference / radius
        positionConversionFactor = TAU
        velocityConversionFactor = TAU / 60
    }

    private val drivingPIDController = drivingSpark.pidController.apply {
        setFeedbackDevice(drivingEncoder)
        constants = DRIVING_PID_COEFFICIENTS
        ff = DRIVING_FF
    }

    private val turningPIDController = turningSpark.pidController.apply {
        setFeedbackDevice(turningEncoder)
        constants = TURNING_PID_COEFFICIENTS

        // enable PID wrapping so that the controller will go across zero to the setpoint
        positionPIDWrappingEnabled = true
        positionPIDWrappingMinInput = 0.0
        positionPIDWrappingMaxInput = TAU
    }


    internal companion object Constants {
        // MAXSwerve can be configured with different pinion gears to make the module faster or increase torque
        val DRIVING_MOTOR_PINION_TEETH = 14

        // The gear ratio between the motor and the wheel.
        // I.e. the wheel angle divided by the motor angle.
        // Motor Pinion : Motor Spur Gear = x : 22
        // Bevel Pinion : Wheel Bevel Gear = 15 : 45
        val DRIVING_MOTOR_TO_WHEEL_GEARING = (DRIVING_MOTOR_PINION_TEETH.toDouble() / 22.0) * (15.0 / 45.0)

        // The free speed of the motor in Rotation2d per second
        val DRIVING_MOTOR_FREE_SPEED = Rotation2d.fromRotations(5760.0 / 60.0)

        // take the known wheel diameter, divide it by two to get the radius, then get the circumference
        val WHEEL_RADIUS = Units.inchesToMeters(3.0) / 2
        val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

        val DRIVING_PID_COEFFICIENTS = PIDCoefficients(p = 0.04)
        val DRIVING_FF = 1 / DRIVING_MOTOR_FREE_SPEED.rotations

        val TURNING_PID_COEFFICIENTS = PIDCoefficients(p = 2.0)

        val DRIVING_CURRENT_LIMIT = 50
        val TURNING_CURRENT_LIMIT = 20
    }
}

class SimSwerveModuleIO : ModuleIO{

    override fun updateInputs(inputs: ModuleInputs) {
        inputs.state = SwerveModuleState(
            drivingMotor.angularVelocityRadPerSec * MAXSwerveModuleIO.WHEEL_RADIUS,
            Rotation2d.fromRadians(turningMotor.angularPositionRad)
        )
        inputs.desiredState = SwerveModuleState.optimize(SwerveModuleState(), inputs.state.angle)

        inputs.position = SwerveModulePosition(
            drivingMotor.angularPositionRad * MAXSwerveModuleIO.WHEEL_RADIUS,
            Rotation2d.fromRadians(turningMotor.angularPositionRad)
        )

    }

    // TODO: figure out what the moment of inertia actually is and if it even matters
    private val turningMotor = DCMotorSim(DCMotor.getNEO(1), TAU, 0.0001)
    private val drivingMotor = DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025)

    private val drivingFeedforward = SimpleMotorFeedforward(0.116970, 0.133240)
    private val drivingFeedback = PIDController(PIDCoefficients(p = 0.9))

    private val turningFeedback = PIDController(PIDCoefficients(p = 7.0)).apply {
        enableContinuousInput(0.0, TAU)
    }

    override fun periodic(inputs: ModuleInputs) {
        // TODO: there should maybe be a proper dt here instead
        // Update the flywheel simulations.
        turningMotor.update(Robot.period)
        drivingMotor.update(Robot.period)

        // Set the new input voltages
        turningMotor.setInputVoltage(
            turningFeedback.calculate(inputs.state.angle.radians, inputs.desiredState.angle.radians)
        )
        drivingMotor.setInputVoltage(
            drivingFeedforward.calculate(inputs.desiredState.speedMetersPerSecond) + drivingFeedback.calculate(
                inputs.state.speedMetersPerSecond, inputs.desiredState.speedMetersPerSecond
            )
        )
    }
}

