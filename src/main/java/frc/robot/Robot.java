// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.framework.TimedRobot;
import org.wpilib.system.DataLogManager;

// import org.wpilib.command3.Command;
// import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
// import org.wpilib.command3.Scheduler;
// import org.wpilib.command2.sysid.Scheduler;;;
// import org.wpilib.driverstation.MatchState;
// import org.wpilib.driverstation.RobotState;
// import org.wpilib.driverstation.Alliance;
// import org.wpilib.driverstation.MatchType;
// import org.wpilib.driverstation.DriverStationErrors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  // private final RobotContainer m_robotContainer = new RobotContainer();

  public Robot()
  { 
    // Start recording to data log
    DataLogManager.start();
    System.out.println("Hi");

    // Record DS control and joystick data.
    // Change to `false` to not record joystick data.
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */

  @Override
  public void autonomousInit() {
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  
  // private SparkFlex a = new SparkFlex(0, 3, MotorType.kBrushless);
  // private SparkFlex b = new SparkFlex(0, 4, MotorType.kBrushless);
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // a.configure(Configs.claw, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // b.configure(Configs.claw, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    // a.setThrottle(0.1);
    // b.setThrottle(0.05);
  }

  @Override
  public void teleopExit() {}

  public void utilityInit() {
    // Cancels all running commands at the start of test mode.
  }

  /** This function is called periodically during test mode. */
  public void utilityPeriodic() {}

  public void utilityExit() {}
}