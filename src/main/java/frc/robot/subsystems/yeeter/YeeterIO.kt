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
    class YeeterRawInputs : LoggableInputs {

        var mainVelocity = 0.0
        var secondaryVelocity = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Yeet Main Motor Velocity", mainVelocity)
            table?.put("Yeet Secondary Motor Velocity", secondaryVelocity)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Main Motor Velocity", mainVelocity)?.let { mainVelocity = it }
            table?.getDouble("Secondary Motor Velocity", secondaryVelocity)?.let { secondaryVelocity = it }
        }

    }


    fun updateInputs(inputs: YeeterInputs)

    fun setSpeedMain(speed: Double)
    fun setSpeedSecondary(speed: Double)

    fun setVoltageMain(outputVolts: Double)
    fun setVoltageSecondary(outputVolts: Double)

    fun getSpeedMain(): Double
    fun getSpeedSecondary(): Double
}

class YeeterIOReal(mainMotorCAN: CANDevice, secondaryMotorCAN: CANDevice) : YeeterIO {


    private val mainMotor = WPI_TalonFX(secondaryMotorCAN.id)
    private val secondaryMotor = CANSparkMax(mainMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        secondaryEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
        secondaryMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: YeeterInputs) {
        inputs.mainVelocity = Rotation2d(mainMotor.get())
        inputs.secondaryVelocity = Rotation2d(secondaryEncoder.velocity)
        inputs.updateRaw()

    }

    override fun setSpeedMain(speed: Double) {
        mainMotor.set(speed)
    }

    override fun setVoltageMain(outputVolts: Double) {
        mainMotor.setVoltage(outputVolts)
    }

    override fun setSpeedSecondary(speed: Double) {
        secondaryMotor.set(speed)
    }

    override fun setVoltageSecondary(outputVolts: Double) {
        secondaryMotor.setVoltage(outputVolts)
    }

    override fun getSpeedMain(): Double {
        return mainMotor.get()
    }

    override fun getSpeedSecondary(): Double {
        return secondaryMotor.get()
    }

    internal companion object Constants {
        const val SECONDARY_GEAR_RATIO = 1.0

    }
}

class YeeterInputs {
    private val inputs = YeeterIO.YeeterRawInputs()

    var mainVelocity = Rotation2d()
    var secondaryVelocity = Rotation2d()

    fun updateRaw() {
        inputs.mainVelocity = mainVelocity.radians
        inputs.secondaryVelocity = secondaryVelocity.radians
    }
}