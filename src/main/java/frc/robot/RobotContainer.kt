package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
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
    private val controller = XboxController(2)


    init {
        configureBindings()
        setDefaultCommands()
        DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation())
    }

    private fun setDefaultCommands(){
        Drivetrain.defaultCommand =
            DriveWithJoysticks(translationJoystick = joystickLeft, rotationJoystick = joystickRight)
        Turret.defaultCommand = Turret.controlWithJoysticks({ controller.leftX }, { controller.leftY })
        Indexer
        Shooter
        BallIntake
    }


    private fun configureBindings() {
        Trigger { Indexer.objectDetected }
            .onTrue(Indexer.AutoIndexCommand())

        JoystickButton(controller, XboxController.Button.kY.value)
            .whileTrue(Indexer.manualIndexCommand)

        JoystickButton(controller, XboxController.Button.kA.value)
            .onTrue(InstantCommand({
                Shooter.spin(1.0)
            }, Shooter))
            .onFalse(InstantCommand({
                Shooter.spin(0.0)
            }, Shooter))

        JoystickButton(controller, XboxController.Button.kB.value)
            .onTrue(InstantCommand({
                Shooter.feed(1.0)
            }, Shooter))
            .onFalse(InstantCommand({
                Shooter.feed(0.0)
            }, Shooter))

        JoystickButton(controller, XboxController.Button.kX.value)
            .onTrue(InstantCommand({
                BallIntake.runRollers(1.0)
            }, BallIntake))
            .onFalse(InstantCommand({
                BallIntake.runRollers(0.0)
            }, BallIntake))

        Trigger { controller.rightTriggerAxis > 0.1 }
            .onTrue(SetIntakePosition(BallIntake.Position.Up.pose, BallIntake))

        Trigger { controller.leftTriggerAxis > 0.1 }
            .onTrue(SetIntakePosition(BallIntake.Position.Down.pose, BallIntake))

        JoystickButton(controller, XboxController.Button.kLeftBumper.value)
            .onTrue(SetIntakePosition(BallIntake.Position.Stowed.pose, BallIntake))
    }

    val autonomousCommand: Command? = null


}
