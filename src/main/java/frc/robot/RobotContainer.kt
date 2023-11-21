package frc.robot

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.XboxController.Button
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.DriveWithJoysticks
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.turret.Turret
import frc.robot.commands.*
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.shooter.Shooter
import java.time.Instant


object RobotContainer {
    // Shuffleboard
    val field = Field2d().also { SmartDashboard.putData("Field", it) }
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)
    private val controller = XboxController(2)


    init {
        configureBindings()
        setDefaultCommands()
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation())
    }

    private fun setDefaultCommands(){
        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        Indexer.defaultCommand = AutoIndex()
        Turret.defaultCommand = Command { setOf(Turret) }
//        Shooter.defaultCommand = object : Command {
//            override fun initialize() {
//                println("Settoing motor speeds")
//                val talon = WPI_TalonFX(11)
//                talon.setNeutralMode(NeutralMode.Coast)
//
//                talon.set(ControlMode.PercentOutput, 1.0)
//            }
//            override fun getRequirements() = setOf(Shooter)
//        }
    }


    private fun configureBindings() {

        JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(
                Turret.setTargetCommand(Rotation2d.fromDegrees(0.0))
        )

        JoystickButton(controller, XboxController.Button.kRightBumper.value).onTrue(
                Turret.setTargetCommand(Rotation2d.fromDegrees(90.0))
        )
       Trigger {
           controller.rightTriggerAxis > 0.05
       }.onTrue(
            Turret.setTargetCommand(Rotation2d.fromDegrees(180.0))
        )
        Trigger {
            controller.leftTriggerAxis > 0.05
        }.onTrue(
                Turret.setTargetCommand(Rotation2d.fromDegrees(270.0))
        )


        JoystickButton(controller, XboxController.Button.kA.value)
                .onTrue(
                        InstantCommand({
                            println("debug: kA detected")
                            Shooter.spin(1.0)
                        }))
                .onFalse(
                        InstantCommand({
                            println("debug: kA released")
                            Shooter.spin(0.0)
                        })
                        )
        JoystickButton(controller, XboxController.Button.kB.value)
                .onTrue(
                        InstantCommand({
                            println("debug: kB detected")
                            Shooter.feed(-1.0)
                        }).alongWith(
                            InstantCommand({
                                Indexer.setSpeed(1.0)
                            })
                        ))
                .onFalse(
                        InstantCommand({
                            println("debug: kB released")
                            Shooter.feed(0.0)
                        }).alongWith(
                                InstantCommand({
                                    Indexer.setSpeed(0.0)
                                })
                        ))


    }

    val autonomousCommand: Command? = null


}