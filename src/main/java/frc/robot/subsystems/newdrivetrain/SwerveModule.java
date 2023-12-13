package frc.robot.subsystems.newdrivetrain;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    SwerveModulePosition getPosition();
    void setDesiredState(SwerveModuleState desiredState);

    SwerveModuleState getState();

    SwerveModuleState getSetState();

    // TODO: use Rotation2d
    double getSwerveEncoderPosition();

    void resetEncoders();

    void update();
}