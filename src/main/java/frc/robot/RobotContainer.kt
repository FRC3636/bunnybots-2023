package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.DriveWithJoysticks
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.turret.Turret
import frc.robot.commands.*


object RobotContainer {
    // Shuffleboard
    val field = Field2d().also { SmartDashboard.putData("Field", it) }
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)
    private val controller = XboxController(2)


    init {
        configureBindings()
    
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());
    }

    private fun setDefaultCommands(){
        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        Turret.defaultCommand = TrackPrimary()
    }


    private fun configureBindings() {






    }

    val autonomousCommand: Command? = null


}