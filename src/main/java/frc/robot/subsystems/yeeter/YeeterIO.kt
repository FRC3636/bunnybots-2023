package frc.robot.subsystems.yeeter

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface YeeterIO {
    class Inputs : LoggableInputs {
        var mainVelocity = Rotation2d()
        var secondaryVelocity = Rotation2d()

        override fun toLog(table: LogTable?) {
            table?.put("Yeet Main Motor Velocity", mainVelocity.radians)
            table?.put("Yeet Secondary Motor Velocity", secondaryVelocity.radians)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Main Motor Velocity", 0.0)?.let { mainVelocity = Rotation2d.fromRadians(it) }
            table?.getDouble("Secondary Motor Velocity", 0.0)?.let { secondaryVelocity = Rotation2d.fromRadians(it) }
        }

    }


    fun updateInputs(inputs: Inputs)

    fun setVoltageMain(outputVolts: Double)
    fun setVoltageSecondary(outputVolts: Double)
}

class YeeterIOReal(mainMotorCAN: CANDevice, secondaryMotorCAN: CANDevice) : YeeterIO {
    private val mainMotor = WPI_TalonFX(secondaryMotorCAN.id)
    private val secondaryMotor = CANSparkMax(mainMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        secondaryEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
        secondaryMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: YeeterIO.Inputs) {
        inputs.mainVelocity = Rotation2d(mainMotor.get())
        inputs.secondaryVelocity = Rotation2d(secondaryEncoder.velocity)
    }

    override fun setVoltageMain(outputVolts: Double) {
        mainMotor.setVoltage(outputVolts)
    }

    override fun setVoltageSecondary(outputVolts: Double) {
        secondaryMotor.setVoltage(outputVolts)
    }

    internal companion object Constants {
        const val SECONDARY_GEAR_RATIO = 1.0
    }
}