package frc.robot.subsystems.intake


import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.robot.CANDevice


class BallIntakeIOReal : IntakeIOReal(CANDevice.BallIntakeArmMotor.id, CANDevice.BallIntakeRollerMotor.id, GEAR_RATIO) {

    internal companion object Constants {
        const val GEAR_RATIO = 1.0
    }

}

class BallIntakeIOSim(): IntakeIO {
    companion object {
        const val INTAKE_LENGTH_M = 0.383
        const val INTAKE_MASS_KG = 6.0
        const val ROLLER_INERTIA_KG_M2 = 0.000226
        const val ROLLER_GEAR_RATIO = 36.0 / 12.0
        const val GEAR_RATIO = 112.5
    }

    private val armMotor = SingleJointedArmSim(
        DCMotor.getNEO(1),
        GEAR_RATIO,
        SingleJointedArmSim.estimateMOI(INTAKE_LENGTH_M, INTAKE_MASS_KG),
        INTAKE_LENGTH_M,
        Units.degreesToRadians(-10.0),
        Units.degreesToRadians(100.0),
        true
    )
    private val rollerMotor = FlywheelSim(
        DCMotor.getNEO(1),
        ROLLER_GEAR_RATIO,
        ROLLER_INERTIA_KG_M2,
    )
    private var frozen = true

    override fun updateInputs(inputs: IntakeIO.Inputs){
        if (!frozen) {
            armMotor.update(0.02)
            rollerMotor.update(0.02)
        }
        inputs.position = Rotation2d(armMotor.angleRads)
        inputs.armVelocity = Rotation2d(armMotor.velocityRadPerSec)
        inputs.rollersOn = rollerMotor.currentDrawAmps > 0
    }

    override fun setArmVoltage(outputVolts: Double){
        frozen = false
        armMotor.setInputVoltage(outputVolts.coerceIn(-12.0, 12.0))
    }


    override fun setRollerSpeed(speed: Double){
        rollerMotor.setInputVoltage(speed.coerceIn(-1.0, 1.0) * 12.0)
    }
}
