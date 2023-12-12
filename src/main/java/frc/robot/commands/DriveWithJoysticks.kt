package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.drivetrain.Drivetrain

class DriveWithJoysticks(private val translationJoystick: Joystick, private val rotationJoystick: Joystick) : Command {
    override fun getRequirements() = setOf(Drivetrain)

    override fun execute() {
        Drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationJoystick.y * MAX_SPEED_METERS_PER_SECOND,
                translationJoystick.x * MAX_SPEED_METERS_PER_SECOND,
                rotationJoystick.x * MAX_ROTATIONAL_SPEED,
                Rotation2d()
            )
        )
    }

    internal companion object Constants {
        //might need to be like 14.75 for the max speed or smth like that
        const val MAX_SPEED_METERS_PER_SECOND = 4.8
        const val MAX_ROTATIONAL_SPEED = 2*Math.PI
    }
}
