package frc.robot.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.drivetrain.Drivetrain

class DriveWithJoysticks(private val translationJoystick: Joystick, private val rotationJoystick: Joystick) : Command {
    override fun getRequirements() = setOf(Drivetrain)

    override fun execute() {
        Drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationJoystick.y,
                translationJoystick.x,
                rotationJoystick.x,
                Drivetrain.rotation
            )
        )
    }
}
