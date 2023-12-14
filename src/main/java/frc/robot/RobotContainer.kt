package frc.robot

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.DriveWithJoysticks
import frc.robot.commands.SetIntakePosition
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.intake.BallIntake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.turret.Turret
import java.time.Instant
import kotlin.math.atan2


object RobotContainer {
    // Shuffleboard
    val field = Field2d().also { SmartDashboard.putData("Field", it) }
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)
    private val simJoystick = if (RobotBase.isSimulation()) {
        Joystick(3)
    } else {
        null
    }
    private val controller = XboxController(2)


    init {
        configureBindings()
        setDefaultCommands()
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation())
    }

    private fun setDefaultCommands() {

        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        //  Turret.defaultCommand = Turret.trackPrimaryTarget()
        Indexer
        Shooter
        BallIntake
//        Turret.defaultCommand = InstantCommand().also {it.addRequirements(Turret)}

    }


    private fun configureBindings() {
        // Driver bindings

        JoystickButton(joystickLeft, 1).whileTrue(
            InstantCommand({
                Shooter.spin(1.0)
            }).also { it.addRequirements(Shooter) }
        ).whileFalse(
            InstantCommand({
                Shooter.spin(0.2)
            }, Shooter)
        )

        JoystickButton(joystickLeft, 8).onTrue(
            InstantCommand({
                println(">>> ZEROING GYRO!!! hi driver :D <<<")
                Drivetrain.zeroGyro()
            })
        )

        JoystickButton(joystickRight, 1)
            .onTrue(InstantCommand({
                Shooter.feed(1.0)
            }))
            .onFalse(InstantCommand({
                Shooter.feed(0.0)
            }))

        // Operator bindings

//        Trigger { controller.leftX > 0.1 || controller.leftX > 0.1 }.whileTrue(
//            Turret.controlWithJoysticks({ controller.leftX }, { controller.leftY })
//        )

        Trigger { controller.leftTriggerAxis >= 0.5 }
            .onTrue(
                Indexer.setSpeedCommand(-1.0)
            )
            .onFalse(
                Indexer.setSpeedCommand(0.0))

        JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(
            BallIntake.runRollersCommand(-1.0)
        ).onFalse(
            BallIntake.runRollersCommand(0.0)
        )

        JoystickButton(controller, XboxController.Button.kRightBumper.value)
            .onTrue(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(1.0),
                    SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
                )
            ).onFalse(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(0.0),
                    SetIntakePosition(BallIntake.Position.Up.pose, BallIntake),
                )
            )

        JoystickButton(controller, XboxController.Button.kB.value)
            .onTrue(
                Indexer.setSpeedCommand(1.0)
            ).onFalse(
                Indexer.setSpeedCommand(0.0)
            )

        Trigger { controller.pov == 90 }
            .onTrue(
                SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
            )

        Trigger { controller.pov == 270 }
            .onTrue(
                SetIntakePosition(BallIntake.Position.Up.pose, BallIntake),
            )
    }

    val autonomousCommand: Command? = null


}
