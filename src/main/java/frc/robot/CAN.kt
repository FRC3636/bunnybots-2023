package frc.robot

enum class CANDevice(val id: Int) {

    //swerve
    FrontLeftTurningMotor(4),
    FrontLeftDrivingMotor(3),
    FrontRightTurningMotor(6),
    FrontRightDrivingMotor(5),
    BackLeftTurningMotor(2),
    BackLeftDrivingMotor(1),
    BackRightTurningMotor(8),
    BackRightDrivingMotor(7),

    IndexerMotor(17),

    TurretMotor(18),

    FlywheelMotor(19),
    ShooterFeedMotor(20),

    BallIntakeRollerMotor(21),
    BallIntakeArmMotor(22)
}