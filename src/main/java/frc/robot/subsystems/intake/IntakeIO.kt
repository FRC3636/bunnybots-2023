package frc.robot.subsystems.intake

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import com.revrobotics.SparkMaxAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface IntakeIO  {

    class Inputs : LoggableInputs {
        var position = Rotation2d()
        var armVelocity = Rotation2d()
        var rollersOn = false
        var armVoltage: Double = 0.0

        override fun toLog(table: LogTable?) {
            table?.put("Intake Arm Position", position.radians)
            table?.put("Intake Arm Speed", armVelocity.radians)
            table?.put("Intake Roller State", rollersOn)
            table?.put("Intake Arm Voltage", armVoltage)
        }


        override fun fromLog(table: LogTable?) {
            table?.getDouble("Intake Arm Position", position.radians)?.let { position = Rotation2d(it) }
            table?.getDouble("Intake Arm Speed", armVelocity.radians)?.let { armVelocity = Rotation2d(it) }
            table?.getBoolean("Intake Roller State", rollersOn)?.let{ rollersOn = it }
            table?.getDouble("Intake Arm Voltage", armVoltage)?.let { armVoltage = it }
        }
    }

    fun updateInputs(inputs: Inputs)

    fun setArmVoltage(outputVolts: Double) {}

    fun setRollerSpeed(speed: Double)

}


abstract class IntakeIOReal(armMotorID: Int, rollersMotorID: Int, gearRatio:Double): IntakeIO {

    private val armMotor = CANSparkMax(armMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val armEncoder = armMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
    private val rollerMotor = CANSparkMax(rollersMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        armEncoder.positionConversionFactor = Units.rotationsToRadians(1.0) *  gearRatio
        armEncoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * gearRatio / 60
        armMotor.encoder.velocityConversionFactor = Units.rotationsToRadians(1.0) * gearRatio / 60
        armMotor.encoder.positionConversionFactor = Units.rotationsToRadians(1.0) * gearRatio / 60

        rollerMotor.inverted = true
        armMotor.burnFlash()
    }

    override fun updateInputs(inputs: IntakeIO.Inputs){
        inputs.position = Rotation2d(armEncoder.position)
        inputs.armVelocity = Rotation2d(armEncoder.velocity)
        inputs.rollersOn = rollerMotor.appliedOutput > 0
        inputs.armVoltage = armMotor.appliedOutput * armMotor.busVoltage

    }

    override fun setArmVoltage(outputVolts: Double){
        armMotor.setVoltage(outputVolts)
    }


    override fun setRollerSpeed(speed: Double){
        rollerMotor.set(speed)
    }

}
