package frc.robot

enum class CANDevice(val id: Int) {
    FrontLeftTurningMotor(10),
    FrontLeftDrivingMotor(11),
    FrontRightTurningMotor(12),
    FrontRightDrivingMotor(13),
    BackLeftTurningMotor(14),
    BackLeftDrivingMotor(15),
    BackRightTurningMotor(16),
    BackRightDrivingMotor(17),
    TurretMotor(18),
    YeetWheelMotor(19),
    YeetFeedMotor(20),
    IntakeFeedMotor(21),
    IntakeAngleMotor(22)
}