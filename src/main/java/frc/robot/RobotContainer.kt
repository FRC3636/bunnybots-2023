package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.commands.DriveWithJoysticks
import frc.robot.commands.SetIntakePosition
import frc.robot.subsystems.drivetrain.Drivetrain
import frc.robot.subsystems.indexer.Indexer
import frc.robot.subsystems.intake.BallIntake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.targetvision.TargetVision
import frc.robot.subsystems.turret.Turret
import frc.robot.utils.LimelightHelpers
import org.littletonrobotics.junction.Logger
import kotlin.math.pow
import kotlin.math.sign


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

        LimelightHelpers.setCameraMode_Driver("limelight");
        LimelightHelpers.setLEDMode_ForceOff("limelight")
    }

    private fun setDefaultCommands() {

        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        //  Turret.defaultCommand = Turret.trackPrimaryTarget()
        Indexer
        Shooter
        BallIntake
        TargetVision
        Turret.defaultCommand = object : Command {
            override fun execute() {
                Turret.setSpeed(controller.leftX.pow(2) * controller.leftX.sign * -3.0)
                Logger.getInstance().recordOutput("Turret/input", controller.leftX)
            }

            override fun isFinished(): Boolean {
                return false
            }

            override fun getRequirements() = setOf(Turret)
        }

    }


    private fun configureBindings() {
        // Driver bindings

//        JoystickButton(joystickLeft, 1).whileTrue(
//            InstantCommand({
//                Shooter.spin(1.0)
//            }).also { it.addRequirements(Shooter) }
//        ).whileFalse(
//            InstantCommand({
//                Shooter.spin(0.2)
//            }, Shooter)
//        )

        JoystickButton(joystickLeft, 8).onTrue(
            InstantCommand({
                println(">>> ZEROING GYRO!!! hi driver :D <<<")
                Drivetrain.zeroGyro()
            })
        )

        // Operator bindings

        Trigger { controller.rightTriggerAxis >= 0.5 }
            .onTrue(InstantCommand({
                Shooter.feed(1.0)
                Indexer.setSpeed(1.0)
            }))
            .onFalse(InstantCommand({
                Shooter.feed(0.0)
                Indexer.setSpeed(0.0)
            }))

        Trigger { controller.leftTriggerAxis >= 0.5 }
            .onTrue(
                Indexer.setSpeedCommand(-1.0)
            )
            .onFalse(
                Indexer.setSpeedCommand(0.0)
            )

        JoystickButton(controller, XboxController.Button.kLeftBumper.value)
            .onTrue(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(0.25),
                    SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
                )
            )
            .onFalse(
                BallIntake.runRollersCommand(0.0)
            )

        JoystickButton(controller, XboxController.Button.kRightBumper.value)
            .onTrue(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(1.0),
                    SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
                    Indexer.setSpeedCommand(1.0)
                )
            ).onFalse(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(0.0),
                    SetIntakePosition(BallIntake.Position.Up.pose, BallIntake),
                    Indexer.setSpeedCommand(0.0)
                )
            )

        JoystickButton(controller, XboxController.Button.kA.value)
            .onTrue(
                Indexer.setSpeedCommand(1.0)
            ).onFalse(
                Indexer.setSpeedCommand(0.0)
            )

        JoystickButton(controller, XboxController.Button.kB.value).whileTrue(
            InstantCommand({
                Shooter.spin(1.0)
            }).also { it.addRequirements(Shooter) }
        ).whileFalse(
            InstantCommand({
                Shooter.spin(0.2)
            }, Shooter)
        )

        JoystickButton(controller, XboxController.Button.kX.value)
            .onTrue(
                InstantCommand({
                    Turret.mode = Turret.TurretMode.Zero
                })
            ).onFalse(
                InstantCommand({
                    Turret.mode = Turret.TurretMode.Manual
                })
            )

        JoystickButton(controller, XboxController.Button.kY.value)
            .onTrue(
                InstantCommand({
                    Turret.mode = Turret.TurretMode.Follow
                })
            ).onFalse(
                InstantCommand({
                    Turret.mode = Turret.TurretMode.Manual
                })
            )

        Trigger { controller.pov == 180 }
            .onTrue(
                SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
            )

        Trigger { controller.pov == 0 }
            .onTrue(
                SetIntakePosition(BallIntake.Position.Up.pose, BallIntake),
            )

        Trigger { controller.rightTriggerAxis >= 0.1 }
            .onTrue(
                SequentialCommandGroup(
                    BallIntake.runRollersCommand(0.25),
                    SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
                )
            )
            .onFalse(
                BallIntake.runRollersCommand(0.0)
            )
    }

    val autonomousCommand: Command? = null


}
