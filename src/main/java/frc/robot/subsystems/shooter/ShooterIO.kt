package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ShooterIO {
    class ShooterInputs : LoggableInputs {

        var flywheelVelocity: Rotation2d = Rotation2d()
        var feederVelocity: Rotation2d = Rotation2d()

        override fun toLog(table: LogTable?) {
            table?.put("Yeet Flywheel Motor Velocity", flywheelVelocity.radians)
            table?.put("Yeet Secondary Motor Velocity", feederVelocity.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Flywheel Motor Velocity", flywheelVelocity.radians)
                ?.let { flywheelVelocity = Rotation2d(it) }
            table?.getDouble("Secondary Motor Velocity", feederVelocity.radians)
                ?.let { feederVelocity = Rotation2d(it) }
        }

    }


    fun updateInputs(inputs: ShooterInputs)

    fun setSpeedFlywheel(speed: Double)
    fun setSpeedFeeder(speed: Double)

    fun setVoltageFlywheel(outputVolts: Double)
    fun setVoltageFeeder(outputVolts: Double)

    fun getSpeedFlywheel(): Double
    fun getSpeedFeeder(): Double
}

class ShooterIOReal(flywheelMotorCAN: CANDevice, feedMotorCAN: CANDevice) : ShooterIO {


    private val FlywheelMotor = WPI_TalonFX(feedMotorCAN.id)
    private val secondaryMotor = CANSparkMax(flywheelMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        secondaryEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
        secondaryMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: ShooterIO.ShooterInputs) {
        inputs.flywheelVelocity = Rotation2d(FlywheelMotor.get())
        inputs.feederVelocity = Rotation2d(secondaryEncoder.velocity)
    }

    override fun setSpeedFlywheel(speed: Double) {
        FlywheelMotor.set(speed)
    }

    override fun setVoltageFlywheel(outputVolts: Double) {
        FlywheelMotor.setVoltage(outputVolts)
    }

    override fun setSpeedFeeder(speed: Double) {
        secondaryMotor.set(speed)
    }

    override fun setVoltageFeeder(outputVolts: Double) {
        secondaryMotor.setVoltage(outputVolts)
    }

    override fun getSpeedFlywheel(): Double {
        return FlywheelMotor.get()
    }

    override fun getSpeedFeeder(): Double {
        return secondaryMotor.get()
    }

    internal companion object Constants {
        const val SECONDARY_GEAR_RATIO = 1.0

    }
}