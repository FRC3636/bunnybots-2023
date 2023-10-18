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

        var yeetMainVelocity = 0.0
        var yeetSecondaryVelocity = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Yeet Main Motor Velocity", yeetMainVelocity)
            table?.put("Yeet Secondary Motor Velocity", yeetSecondaryVelocity)
        }

        override fun fromLog(table: LogTable?) {
            table?.getDouble("Main Motor Velocity", yeetMainVelocity)?.let {yeetMainVelocity = it}
            table?.getDouble("Secondary Motor Velocity", yeetSecondaryVelocity)?.let {yeetSecondaryVelocity = it}
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

class YeeterIOReal(yeetMainMotorCAN: CANDevice, yeetSecondaryMotorCAN: CANDevice) : YeeterIO {


    private val yeetMainMotor = WPI_TalonFX(yeetSecondaryMotorCAN.id)
    private val secondaryMotor = CANSparkMax(yeetMainMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        secondaryEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
        secondaryMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * SECONDARY_GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: YeeterInputs) {
        inputs.yeetMainVelocity = Rotation2d(yeetMainMotor.get())
        inputs.yeetSecondaryVelocity = Rotation2d(secondaryEncoder.velocity)
        inputs.updateRaw()

    }

    override fun setSpeedMain(speed: Double){
        yeetMainMotor.set(speed)
    }

    override fun setVoltageMain(outputVolts: Double){
        yeetMainMotor.setVoltage(outputVolts)
    }

    override fun setSpeedSecondary(speed: Double) {
        secondaryMotor.set(speed)
    }
    override fun setVoltageSecondary(outputVolts: Double) {
        secondaryMotor.setVoltage(outputVolts)
    }

    override fun getSpeedMain(): Double {
        return yeetMainMotor.get()
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

    var yeetMainVelocity = Rotation2d()
    var yeetSecondaryVelocity = Rotation2d()

    fun updateRaw() {
        inputs.yeetMainVelocity = yeetMainVelocity.radians
        inputs.yeetSecondaryVelocity = yeetSecondaryVelocity.radians
    }
}