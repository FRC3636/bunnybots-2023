package frc.robot.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.CANDevice
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

public interface IntakeIO  {

    class IntakeInputsRaw : LoggableInputs {
        var position: Double = 0.0
        var velocity: Double = 0.0
        var rollers: Double = 0.0


            override fun toLog(table: LogTable?) {
                table?.put("Yeet Main Motor Velocity", position)
                table?.put("Yeet Secondary Motor Velocity", velocity)

            }

            override fun fromLog(table: LogTable?) {
                table?.getDouble("Main Motor Velocity", velocity)?.let {velocity = it}
                table?.getDouble("Secondary Motor Velocity", position)?.let {position = it}
            }



    }

    fun updateInputs(inputs: IntakeInputs)

    fun setSpeed(speed: Double)

    fun setVoltage(outputVolts: Double) {}
}

class IntakeInputs {

    val inputs: IntakeIO.IntakeInputsRaw = IntakeIO.IntakeInputsRaw()
    var position: Rotation2d = Rotation2d()
    var velocity: Rotation2d = Rotation2d()
    var rollers: Rotation2d = Rotation2d()

    fun updateRaw(){
        inputs.position = position.radians
        inputs.velocity = velocity.radians
        inputs.rollers = rollers.radians

    }
}


class IntakeIOReal(PrimaryMotorCAN: CANDevice, SecondaryMotorCAN: CANDevice) : IntakeIO {


    private val motor = CANSparkMax(PrimaryMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val secondaryMotor = CANSparkMax(SecondaryMotorCAN.id, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val secondaryEncoder = secondaryMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)


    init {
        encoder.positionConversionFactor = Units.rotationsToRadians(1.0) *  GEAR_RATIO
        encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
        motor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * GEAR_RATIO / 60
    }

    override fun updateInputs(inputs: IntakeInputs){
        inputs.position = Rotation2d(encoder.position)
        inputs.velocity = Rotation2d(encoder.velocity)
        inputs.updateRaw()
        TODO("Not yet implemented")
    }

    override  fun setSpeed(speed: Double){
        motor.set(speed)
    }

    override fun setVoltage(outputVolts: Double){
        motor.setVoltage(outputVolts)
    }


    internal companion object Constants {
        const val GEAR_RATIO = 1.0

    }
}
