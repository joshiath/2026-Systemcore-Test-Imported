// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import org.wpilib.command3.Command;
// import org.wpilib.command2.Command;
// import org.wpilib.command3.Mechanism;
import org.wpilib.command2.SubsystemBase;
// import org.wpilib.command3.Trigger;
import org.wpilib.command2.button.Trigger;
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

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.discrete.DigitalInput;

import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.Constants.Ports;

public class Elevator extends SubsystemBase {
  private SparkFlex motorL = new SparkFlex(CANPort.CAN_S1, Ports.elevatorL, MotorType.kBrushless);
  private SparkFlex motorR = new SparkFlex(CANPort.CAN_S1, Ports.elevatorR, MotorType.kBrushless);

  private ArmFeedforward ff = new ArmFeedforward(0, 0, 0);
  private SparkClosedLoopController controller = motorL.getClosedLoopController();

  private DigitalInput topLimit = new DigitalInput(0);
  private DigitalInput botLimit = new DigitalInput(1);

  /** Creates a new Elevator. */
  public Elevator() 
  {
    motorL.configure(Configs.elevatorL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorR.configure(Configs.elevatorR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public enum ElevatorState
  {
    TRANSITION(0),
    CONESCORE(100),
    CUBESCORE(100),
    CONEINTAKE(70),
    CUBEINTAKE(70);

    public double position;
    private ElevatorState(double position)
    {
      this.position = position;
    } 
  }
  private ElevatorState state = ElevatorState.TRANSITION;

  public void periodic() 
	{
		controller.setSetpoint(state.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, Math.sin(Math.toRadians(RobotContainer.getPivotPose()))*ff.calculate(Math.toRadians(motorL.getAbsoluteEncoder().getPosition().get()), 0));

		Telemetry.log("elevator pos", motorL.getAbsoluteEncoder().getPosition().get());

    // isDown.onTrue(Command.requirements(this).executing(coroutine->{motorL.getEncoder().setPosition(0);}).named("Zero elevator"));
    // isUp.onTrue(Command.requiring(this).executing(coroutine->{motorL.stopMotor();motorR.stopMotor();}).named("Stop elevator"));
	}

  public void transitionState()
  {
    state = ElevatorState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = ElevatorState.CONESCORE;
  }
  public void cubeIntake()
  {
    state = ElevatorState.CUBEINTAKE;
  }
  public void coneIntake()
  {
    state = ElevatorState.CONEINTAKE;
  }
  public void coneScoreState()
  {
    state = ElevatorState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = ElevatorState.CUBESCORE;
  }

  public Trigger isDown = new Trigger(()-> botLimit.get());
  
  public Trigger isUp = new Trigger(()-> topLimit.get());

}
