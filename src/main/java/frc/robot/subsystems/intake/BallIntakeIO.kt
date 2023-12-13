package frc.robot.subsystems.intake


import frc.robot.CANDevice


class BallIntakeIOReal : IntakeIOReal(CANDevice.BallIntakeArmMotor.id, CANDevice.BallIntakeRollerMotor.id, GEAR_RATIO) {

    internal companion object Constants {
        const val GEAR_RATIO = 112.5 // from lewis cad skillz
    }

}
