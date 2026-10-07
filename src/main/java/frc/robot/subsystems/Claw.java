// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
// import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ArmFeedforward;
 import org.wpilib.telemetry.Telemetry;
//  import org.wpilib.smartdashboard.Field2d;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class Claw extends SubsystemBase {
  private SparkFlex tilt = new SparkFlex(CANPort.CAN_S1, Ports.tilt, MotorType.kBrushless);
  private SparkFlex claw = new SparkFlex(CANPort.CAN_S1, Ports.claw, MotorType.kBrushless);

  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private SparkClosedLoopController controller = tilt.getClosedLoopController();

  public Claw() 
  {
    tilt.configure(Configs.tilt, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    claw.configure(Configs.claw, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum TiltState
  {
    TRANSITION(90),
    CUBEINTAKEFRONT(100),
    CUBEINTAKEBACK(100),
    CONEINTAKEFRONT(150),
    CONEINTAKEBACK(150),
    CUBESCORE(20),
    CONESCORE(20);

    public double position;
    private TiltState(double position)
    {
      this.position = position;
    } 
  }
  private TiltState state = TiltState.TRANSITION;

	public void periodic() 
	{
		controller.setSetpoint(
      state.position, 
      ControlType.kPosition, ClosedLoopSlot.kSlot0, 
      ff.calculate(Math.toRadians(tilt.getAbsoluteEncoder().getPosition().get() - RobotContainer.getPivotPose()),
      0));

		Telemetry.log("tilt pos", tilt.getAbsoluteEncoder().getPosition().get());
	}

  public void intake()
  {
    claw.setThrottle(0.5);
  }

  public void outtake()
  {
    claw.setThrottle(-0.5);
  }

  public void stop()
  {
    claw.stopMotor();
  }

  public void transitionState()
  {
    state = TiltState.TRANSITION;
    claw.stopMotor();
  }
  public void scoreConeState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = TiltState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = TiltState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = TiltState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = TiltState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = TiltState.CUBESCORE;
  }
}
