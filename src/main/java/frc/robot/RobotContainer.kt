package frc.robot

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.DriveWithJoysticks
import frc.robot.subsystems.drivetrain.Drivetrain


object RobotContainer {
    // Shuffleboard
    val field = Field2d().also { SmartDashboard.putData("Field", it) }

    init {
        configureBindings()
    
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());
    }

    private fun configureBindings() {
        val joystickLeft = Joystick(0)
        val joystickRight = Joystick(1)

        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
    }

    val autonomousCommand: Command? = null
}