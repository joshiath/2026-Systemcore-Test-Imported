// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

// import org.wpilib.command3.Command;
import org.wpilib.command2.Command;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.RunCommand;
// import org.wpilib.command3.Trigger;
import org.wpilib.command2.button.Trigger;
// import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.command2.button.CommandGamepad;
// import org.wpilib.command3.button.JoystickButton;
import org.wpilib.command2.button.JoystickButton;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

/*
* This class is where the bulk of the robot should be declared.  Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls).  Instead, the structure of the robot
* (including subsystems, commands, and button mappings) should be declared here.
*/

public class RobotContainer {
    // The robot's subsystems
    private static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private static final Pivot m_pivot = new Pivot();
    private static final Elevator m_elevator = new Elevator();
    private static final Claw m_claw = new Claw();

    // The driver's controller
    public static final CommandGamepad m_cont0 = new CommandGamepad(0);

    private final static double defaultSpeed = 0.8;
	private final static double slowSpeed = 0.2;
	private static double speedMod = defaultSpeed;

    
    public static Trigger slow = m_cont0.leftTrigger(0.3);

    public static Trigger cone = m_cont0.button(3);
    public static Trigger cube = cone.negate();

    public static Trigger front = m_cont0.rightTrigger(0.2);

    public static Trigger intaking = m_cont0.rightBumper();

    public static Trigger cubeIntakeFront = cube.and(front).and(intaking).and(slow);
    public static Trigger cubeIntakeBack = cube.and(front).negate().and(intaking).and(slow);

    public static Trigger coneIntakeFront = cone.and(front).and(intaking).and(slow);
    public static Trigger coneIntakeBack = cone.and(front).negate().and(intaking).and(slow);

    public static Trigger coneScoreState = cone.and(intaking).negate().and(slow);
    public static Trigger cubeScoreState = cube.and(intaking).negate().and(slow);

    public static Trigger aligning = m_cont0.leftStick();     

    public static Trigger scoreCone = m_cont0.leftBumper().and(cone).and(slow);
    public static Trigger scoreCube = m_cont0.leftBumper().and(cube).and(slow);

    public static Trigger intakingCone = intaking.and(cone);
    public static Trigger intakingCube = intaking.and(cube);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // CANBus.systemcore(0);
        // CANBus.
        // CANBus.systemcore(1);
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // m_driveSubsystem.setDefaultCommand(
        //     // Command.requiring(m_driveSubsystem).executing(
        //     //     coroutine -> {
        //     //         m_driveSubsystem.drive(
		// 	//         m_driveSubsystem.inputDeadband(-m_cont0.getLeftY()) * speedMod,
        //     //         m_driveSubsystem.inputDeadband(-m_cont0.getLeftX()) * speedMod,
        //     //         m_driveSubsystem.inputDeadband(-m_cont0.getRightX()) * speedMod,
		// 	// true);}
        //     // ).named("defaultDrive")

            m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.drive(
                m_driveSubsystem.inputDeadband(-m_cont0.getLeftY()) * speedMod,
                m_driveSubsystem.inputDeadband(-m_cont0.getLeftX()) * speedMod,
                 m_driveSubsystem.inputDeadband(-m_cont0.getRightX()), 
                 true), m_driveSubsystem));
    
        // m_claw.setDefaultCommand(
        //     Command.requiring(m_claw).executing(
        //         coro -> {
        //             m_claw.transitionState();
        //     }).named("claw default"));
        m_claw.setDefaultCommand(m_claw.run(() -> m_claw.transitionState()));
        // m_pivot.setDefaultCommand(
        //     Command.requiring(m_pivot).executing(
        //         coro -> {
        //             m_pivot.transitionState();
        //     }).named("pivot default"));
        m_pivot.setDefaultCommand(m_pivot.run(() -> m_pivot.transitionState()));
        // m_elevator.setDefaultCommand(
        //     Command.requiring(m_elevator).executing(
        //         coro -> {
        //             m_elevator.transitionState();
        //     }).named("elevator default"));
        m_elevator.setDefaultCommand(m_elevator.run(() -> m_elevator.transitionState()));


            // new Command(
            //     () -> m_driveSubsystem.drive(
            //         -MathUtil.applyDeadband(m_cont0.getLeftY(), OIConstants.kDriveDeadband),
            //         -MathUtil.applyDeadband(m_cont0.getLeftX(), OIConstants.kDriveDeadband),
            //         -MathUtil.applyDeadband(m_cont0.getRightX(), OIConstants.kDriveDeadband),
            //         true),
            //     m_driveSubsystem));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link org.wpilib.driverstation.GenericHID} or one of its
     * subclasses ({@link
     * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // m_cont0.button(0).whileTrue(
        //     Command.requiring(m_driveSubsystem).executing(
        //         coroutine -> {m_driveSubsystem.setX();})
        //         .named("setX"));

        m_cont0.button(0).whileTrue(m_driveSubsystem.run(() -> m_driveSubsystem.setX()));
        
        
        // m_cont0.start().onTrue(
        //     Command.requiring(m_driveSubsystem).executing(
        //         coroutine -> {m_driveSubsystem.zeroHeading();}).
        //         named("setX"));

        m_cont0.start().onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.zeroHeading()));

        // y is for cone
        // m_cont0.button(1).whileTrue(Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {})
        //     .named("conePickup"));

        // m_cont0.button(1).whileTrue(new ParallelCommandGroup(m_claw.))
        
        // x is for cube
        // new Trigger(()-> m_cont0.getLeftX()>0.1).whileTrue(null)
        // slow.onTrue(Command.requiring(m_driveSubsystem).executing(
        //     coro -> {speedMod = slowSpeed;})
        //     .named("slowmode")).onFalse(Command.requiring(m_driveSubsystem).executing(
        //     coro -> {speedMod = defaultSpeed;}).named("defaultspeed"));
        
        slow.onTrue(new InstantCommand(() -> speedMod = slowSpeed));

        slow.whileFalse(transitionState());
        cubeIntakeFront.whileTrue(cubeIntakeFront());
        cubeIntakeBack.whileTrue(cubeIntakeBack());
        coneIntakeFront.whileTrue(coneIntakeFront());
        coneIntakeBack.whileTrue(coneIntakeBack());
        coneScoreState.whileTrue(coneScoreState());
        cubeScoreState.whileTrue(cubeScoreState());

        scoreCone.whileTrue(scoreCone());
        scoreCube.whileTrue(scoreCube());

        // intakingCube.whileTrue(Command.requiring(m_claw).executing(
        //     coro -> {
        //         m_claw.intake();
        //     }).named("intakeCube"));
        intakingCube.whileTrue(m_claw.run(() -> m_claw.intake()));
        // intakingCone.whileTrue(Command.requiring(m_claw).executing(
        //     coro -> {
        //         m_claw.outtake();
        //     }).named("intakeCone"));
        intakingCone.whileTrue(m_claw.run(() -> m_claw.outtake()));
    }

    private Command transitionState()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.transitionState();
        //         m_elevator.transitionState();
        //         m_claw.transitionState();
        //     }).named("conePickup");
        return new InstantCommand(() -> { m_pivot.transitionState(); m_elevator.transitionState(); m_claw.transitionState(); }, m_pivot, m_elevator, m_claw);
    }
    private Command cubeIntakeFront()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.cubeIntakeFront();
        //         m_elevator.cubeIntake();
        //         m_claw.cubeIntakeBack();
        //     }).named("cubeIntakeFront");
        return new InstantCommand(() -> { m_pivot.cubeIntakeFront(); m_elevator.cubeIntake(); m_claw.cubeIntakeFront();}, m_pivot, m_elevator, m_claw);
    }
    private Command cubeIntakeBack()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.cubeIntakeBack();
        //         m_elevator.cubeIntake();
        //         m_claw.cubeIntakeBack();
        //     }).named("cubeIntakeBack");
        return new InstantCommand(() -> { m_pivot.cubeIntakeBack(); m_elevator.cubeIntake(); m_claw.coneIntakeBack(); }, m_pivot, m_elevator, m_claw);
    }
    private Command coneIntakeFront()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.coneIntakeFront();
        //         m_elevator.coneIntake();
        //         m_claw.coneIntakeBack();
        //     }).named("coneIntakeFront");
        return new InstantCommand(() -> { m_pivot.coneIntakeFront(); m_elevator.coneIntake(); m_claw.coneIntakeFront();} , m_pivot, m_elevator, m_claw);
    }
    private Command coneIntakeBack()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.coneIntakeBack();
        //         m_elevator.coneIntake();
        //         m_claw.coneIntakeBack();
        //     }).named("coneIntakeBack");
        return new InstantCommand(() -> { m_pivot.coneIntakeBack(); m_elevator.coneIntake(); m_claw.coneIntakeBack();}, m_pivot, m_elevator, m_claw);
    }
    private Command coneScoreState()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.coneScoreState();
        //         m_elevator.coneScoreState();
        //         m_claw.coneScoreState();
        //     }).named("coneScoreState");
        return new InstantCommand(() -> {m_pivot.coneScoreState(); m_elevator.coneScoreState(); m_claw.coneScoreState();}, m_pivot, m_elevator, m_claw);
    }
    private Command cubeScoreState()
    {
        // return Command.requiring(m_pivot, m_elevator, m_claw).executing(
        //     coro -> {
        //         m_pivot.cubeScoreState();
        //         m_elevator.cubeScoreState();
        //         m_claw.cubeScoreState();
        //     }).named("cubeScoreState");
        return new InstantCommand(() -> {m_pivot.cubeScoreState(); m_elevator.cubeScoreState(); m_claw.cubeScoreState();}, m_pivot, m_elevator, m_claw);
    }
    private Command scoreCone()
    {
        // return Command.requiring(m_claw).executing(
        //     coro -> {
        //         m_claw.intake();
        //     }).named("scoreCone");
        return new RunCommand(() -> m_claw.intake(), m_claw);
    }
    private Command scoreCube()
    {
        // return Command.requiring(m_claw).executing(
        //     coro -> {
        //         m_claw.outtake();
        //     }).named("scoreCube");
        return new RunCommand(() -> m_claw.outtake(), m_claw);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    //   public Command getAutonomousCommand() {
    //     // Create config for trajectory
    //     TrajectoryConfig config = new TrajectoryConfig(
    //         AutoConstants.kMaxSpeedMetersPerSecond,
    //         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics);

    //     // An example trajectory to follow. All units in meters.
    //     Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         config);

    //     var thetaController = new ProfiledPIDController(
    //         AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    //     thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //     // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //         // exampleTrajectory,
    //         // m_driveSubsystem::getPose, // Functional interface to feed supplier
    //         // DriveConstants.kDriveKinematics,

    //         // // Position controllers
    //         // new PIDController(AutoConstants.kPXController, 0, 0),
    //         // new PIDController(AutoConstants.kPYController, 0, 0),
    //         // thetaController,
    //         // m_driveSubsystem::setModuleStates,
    //         // m_driveSubsystem);

    //     // Reset odometry to the starting pose of the trajectory.
    //     m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    //     // Run path followuuing command, then stop at the end.
    //     return swerveControllerCommand.andThen(() -> m_driveSubsystem.drive(0, 0, 0, false));


    public static double getPivotPose()
    {
        return m_pivot.getPose();
    }
}
