// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallIntake extends SubsystemBase {

    private final CANSparkMax hingeMotor = new CANSparkMax(Constants.BallIntake.HINGE_ID, CANSparkMax.MotorType.kBrushless);

    private final CANSparkMax rollerMotor = new CANSparkMax(Constants.BallIntake.ROLLER_ID, CANSparkMax.MotorType.kBrushless);
    
    public Position position = Position.Up;
    public DigitalInput upLimitSwitch = new DigitalInput(Constants.BallIntake.UP_LIMIT_SWITCH_CHANNEL);
    public DigitalInput downLimitSwitch = new DigitalInput(Constants.BallIntake.DOWN_LIMIT_SWITCH_CHANNEL);


  /** Creates a new BallIntake. */
  public BallIntake() {
    hingeMotor.setInverted(false);
    
  }

  @Override
  public void periodic() {
    if(position == Position.Up){
        if(!upLimitSwitch.get()){
            hingeMotor.getEncoder().setPosition(0);
            hingeMotor.set(0);
            hingeMotor.setIdleMode(IdleMode.kBrake);
        }else{
            hingeMotor.setIdleMode(IdleMode.kCoast);
            hingeMotor.set(Constants.BallIntake.HINGE_SPEED);
        }
    }else if(position == Position.Down){
        if(!downLimitSwitch.get()){
            hingeMotor.set(0);
            hingeMotor.setIdleMode(IdleMode.kBrake);
            hingeMotor.getEncoder().setPosition(Constants.BallIntake.INTAKE_DOWN_ROTATION);
        }else{
            hingeMotor.setIdleMode(IdleMode.kCoast);
            hingeMotor.set(-Constants.BallIntake.HINGE_SPEED);
        }

    }
  }

  public void runRollers(double direction){
    rollerMotor.set(direction/2);
  }

  public void stopRollers(){
    rollerMotor.set(0);

  }

  public void setPosition(Position position){
    this.position = position;
  }

  public enum Position{
    Up,
    Down
  }

}
