package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.drivetrain.Drivetrain
import kotlin.math.abs

class DriveWithJoysticks(private val translationJoystick: Joystick, private val rotationJoystick: Joystick) : Command {
    override fun getRequirements() = setOf(Drivetrain)

    override fun execute() {
        val (tx, ty) = getXYWithDeadband(translationJoystick, DEADBAND)
        val (rx, _) = getXYWithDeadband(rotationJoystick, DEADBAND)
        Drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                ty * MAX_SPEED_METERS_PER_SECOND,
                tx * MAX_SPEED_METERS_PER_SECOND,
                rx * MAX_ROTATIONAL_SPEED,
                Drivetrain.gyroInputs.rotation.toRotation2d()
            )
        )
    }

    fun getXYWithDeadband(joystick: Joystick, deadband: Double): DoubleArray {
        // Negative because joysticks are inverted
        val xValue = -joystick.y * (joystick.z + 1) / 2
        val yValue = -joystick.x * (joystick.z + 1) / 2


        // only apply deadzone if both axes are inside
        return if (abs(xValue) < deadband && abs(yValue) < deadband) {
            doubleArrayOf(0.0, 0.0)
        } else doubleArrayOf(xValue, yValue)
    }

    internal companion object Constants {
        //might need to be like 14.75 for the max speed or smth like that
        const val MAX_SPEED_METERS_PER_SECOND = 18.8
        const val MAX_ROTATIONAL_SPEED = 2*Math.PI * 4
        const val DEADBAND = 0.1
    }
}
