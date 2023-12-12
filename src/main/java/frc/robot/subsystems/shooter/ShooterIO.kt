package frc.robot.subsystems.shooter

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface ShooterIO {
    class Inputs : LoggableInputs {

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

    fun updateInputs(inputs: Inputs)

    fun setSpeedFlywheel(speed: Double)

    fun setSpeedFeeder(speed: Double)
}

class ShooterIOReal(flywheelMotorID: Int, feedMotorID: Int) : ShooterIO {

    private val flywheelMotor = WPI_TalonFX(flywheelMotorID)
    private val feedMotor = CANSparkMax(feedMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

    init{
        feedMotor.inverted = true
    }

    override fun updateInputs(inputs: ShooterIO.Inputs) {
        inputs.flywheelVelocity = Rotation2d(flywheelMotor.motorOutputVoltage)
        inputs.feederVelocity = Rotation2d(feedMotor.encoder.velocity)
    }

    override fun setSpeedFlywheel(speed: Double) {
        flywheelMotor.set(speed)
    }

    override fun setSpeedFeeder(speed: Double) {
        feedMotor.set(speed)
    }

    internal companion object Constants {
        const val SECONDARY_GEAR_RATIO = 1.0
    }
}