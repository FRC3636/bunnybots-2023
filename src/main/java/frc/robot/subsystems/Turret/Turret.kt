package frc.robot.subsystems.Turret

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.CANDevice
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.utils.PIDCoefficients
import frc.robot.utils.PIDController


object Turret : Subsystem {

    val pidController = PIDController(PIDCoefficients(0.0,0.0,0.0))

    val feedForward = SimpleMotorFeedforward(0.0,0.0,0.0)

    val io = if(RobotBase.isReal()){
        TurretIOReal(CANDevice.TurretMotor)
    }else{
        TurretIOSim()
    }


    private val turretInputs = TurretInputs()

    // factoring in rotation of drivetrain
    private var targetRotation: Rotation2d = Rotation2d()


    override fun periodic() {

    }

    fun aim() {

        io.setVoltage(
                pidController.calculate(relativeAngle.radians, targetRotation.radians)
                        + feedForward.calculate(Drivetrain.velocity2d.norm)
        )

    }

    fun setTarget(target: Rotation2d){
        targetRotation = target
    }

    val relativeAngle: Rotation2d
        get() = Rotation2d(turretInputs.position.radians + Drivetrain.rotation.radians)






}