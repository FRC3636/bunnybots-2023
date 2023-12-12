package frc.robot

enum class CANDevice(val id: Int) {

    //swerve
    FrontLeftTurningMotor(6),
    FrontLeftDrivingMotor(5),

    FrontRightTurningMotor(4),
    FrontRightDrivingMotor(3),

    BackLeftTurningMotor(8),
    BackLeftDrivingMotor(7),

    BackRightTurningMotor(2),
    BackRightDrivingMotor(1),

    IndexerMotor(13),

    TurretMotor(10),

    FlywheelMotor(11),
    ShooterFeedMotor(12),

    BallIntakeRollerMotor(15),
    BallIntakeArmMotor(14),
}