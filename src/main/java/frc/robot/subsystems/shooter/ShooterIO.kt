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
            table?.put("Shooter Flywheel Motor Velocity", flywheelVelocity.radians)
            table?.put("Shooter Feeder Motor Velocity", feederVelocity.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Shooter Flywheel Motor Velocity", flywheelVelocity.radians)
                ?.let { flywheelVelocity = Rotation2d(it) }
            table?.getDouble("Shooter Feeder Motor Velocity", feederVelocity.radians)
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

class ShooterIOReal(flywheelMotorID: Int, feedMotorID: Int) : ShooterIO {


    private val flywheelMotor = WPI_TalonFX(flywheelMotorID)
    private val feedMotor = CANSparkMax(feedMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)



    override fun updateInputs(inputs: ShooterIO.ShooterInputs) {
        inputs.flywheelVelocity = Rotation2d(flywheelMotor.get())
        inputs.feederVelocity = Rotation2d(feedMotor.encoder.velocity)
    }

    override fun setSpeedFlywheel(speed: Double) {
        flywheelMotor.set(speed)
    }

    override fun setVoltageFlywheel(outputVolts: Double) {
        flywheelMotor.setVoltage(outputVolts)
    }

    override fun setSpeedFeeder(speed: Double) {
        feedMotor.set(speed)
    }

    override fun setVoltageFeeder(outputVolts: Double) {
        feedMotor.setVoltage(outputVolts)
    }

    override fun getSpeedFlywheel(): Double {
        return flywheelMotor.get()
    }

    override fun getSpeedFeeder(): Double {
        return feedMotor.get()
    }

    internal companion object Constants {
        const val SECONDARY_GEAR_RATIO = 1.0

    }
}