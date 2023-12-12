package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
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

object RobotContainer {
    // Shuffleboard
    val field = Field2d().also { SmartDashboard.putData("Field", it) }
    private val joystickLeft = Joystick(0)
    private val joystickRight = Joystick(1)
    private val simJoystick = if(RobotBase.isSimulation()){Joystick(3)}else{null}
    private val controller = XboxController(2)

    init {
        configureBindings()
        setDefaultCommands()
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation())
    }

    private fun setDefaultCommands(){
        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        //  Turret.defaultCommand = Turret.trackPrimaryTarget()
        Indexer
        // Shooter.defaultCommand = InstantCommand({
        //     Shooter.spin(1.0)
        // }).also {it.addRequirements(Shooter)}
        BallIntake
        Turret.defaultCommand = InstantCommand().also {it.addRequirements(Turret)}
    }

    private fun configureBindings() {
        // JoystickButton(controller, XboxController.Button.kY.value)
        //     .whileTrue(Indexer.manualIndexCommand)

        JoystickButton(joystickRight, 1)
            .onTrue(InstantCommand({
                Shooter.feed(1.0)
            }))
            .onFalse(InstantCommand({
            }))

        Trigger {controller.leftX > 0.1 || controller.leftX > 0.1}.whileTrue(
            Turret.controlWithJoysticks({controller.leftX}, {controller.leftY})
        )

        // JoystickButton(simJoystick, 1).onTrue(
        //     InstantCommand({
        //         Turret.setTarget(Rotation2d.fromDegrees(180.0))
        //     })
        // )
        // JoystickButton(simJoystick, 2).onTrue(
        //     InstantCommand({
        //         Turret.setTarget(Rotation2d.fromDegrees(190.0))
        //     })
        // )
        // JoystickButton(simJoystick, 3).onTrue(
        //     InstantCommand({
        //         Turret.setTarget(Rotation2d.fromDegrees(270.0))
        //     })
        // )
        // JoystickButton(simJoystick, 4).onTrue(
        //     InstantCommand({
        //         Turret.setTarget(Rotation2d.fromDegrees(45.0))
        //     })
        // )

        JoystickButton(controller, XboxController.Button.kLeftBumper.value).onTrue(
            InstantCommand({
                BallIntake.runRollers(-1.0)
            }).alongWith(
                InstantCommand({
                    Indexer.setSpeed(-1.0)
                })
            )
        ).onFalse(
            InstantCommand({
                BallIntake.runRollers(0.0)
            }).alongWith(
                InstantCommand({
                    Indexer.setSpeed(0.0)
                })
            )
        )

        JoystickButton(controller, XboxController.Button.kRightBumper.value)
            .onTrue(ParallelCommandGroup(
                SetIntakePosition(BallIntake.Position.Down.pose, BallIntake),
                InstantCommand({
                    BallIntake.runRollers(1.0)
                }),
                InstantCommand({
                    Indexer.setSpeed(1.0)
                })
            )).onFalse(
                SequentialCommandGroup(
                    InstantCommand({
                        BallIntake.runRollers(0.0)
                    })
                    
                    ,
                    SetIntakePosition(BallIntake.Position.Up.pose, BallIntake)
                ).alongWith(Indexer.autoIndexCommand())
            )

        JoystickButton(controller, XboxController.Button.kLeftBumper.value)
            .onTrue(SetIntakePosition(BallIntake.Position.Stowed.pose, BallIntake))
    }

    val autonomousCommand: Command? = null
}
