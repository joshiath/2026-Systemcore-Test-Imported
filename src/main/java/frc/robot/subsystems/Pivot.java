// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import org.wpilib.command3.Mechanism;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.ArmFeedforward;
// import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.telemetry.Telemetry;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.Ports;

public class Pivot extends SubsystemBase {
  private SparkFlex motorL = new SparkFlex(CANPort.CAN_S1, Ports.pivotL, MotorType.kBrushless);
  private SparkFlex motorR = new SparkFlex(CANPort.CAN_S1, Ports.pivotR, MotorType.kBrushless);

  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private SparkClosedLoopController controller = motorL.getClosedLoopController();

  public Pivot() 
  {
    motorL.configure(Configs.pivotL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR.configure(Configs.pivotL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //angle fron zero to horizontal
  private static final double offset = 70;

  public enum PivotState
  {
    TRANSITION(90),
    CONESCORE(60),
    CUBESCORE(60),
    CONEINTAKEFRONT(-45),
    CONEINTAKEBACK(225),
    CUBEINTAKEFRONT(-45),
    CUBEINTAKEBACK(225);

    public double position;
    private PivotState(double position)
    {
      this.position = position + offset;
    } 
  }
  private PivotState state = PivotState.TRANSITION;

	public void periodic() 
	{
		controller.setSetpoint(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff.calculate(Math.toRadians(motorL.getAbsoluteEncoder().getPosition().get() -offset), 0));

		Telemetry.log("pivot pos", motorL.getAbsoluteEncoder().getPosition().get());
	}

  public double getState()
  {
    return state.position;
  }

  public double getPose()
  {
    return motorL.getAbsoluteEncoder().getPosition().get()-offset;
  }

  public void transitionState()
  {
    state = PivotState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = PivotState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = PivotState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = PivotState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = PivotState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = PivotState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = PivotState.CUBESCORE;
  }
}
