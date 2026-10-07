package frc.robot;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {

  public static final SparkFlexConfig pivotL = new SparkFlexConfig();
  public static final SparkFlexConfig pivotR = new SparkFlexConfig();
  public static final SparkFlexConfig elevatorL = new SparkFlexConfig();
  public static final SparkFlexConfig elevatorR = new SparkFlexConfig();
  public static final SparkFlexConfig claw = new SparkFlexConfig();
  public static final SparkFlexConfig tilt = new SparkFlexConfig();

  public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  static {
    // Use module constants to calculate conversion factors and feed forward gain.
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI 
      / ModuleConstants.kDrivingMotorReduction;
    double turningFactor = 2 * Math.PI;

    double nominalVoltage = 12.0;
    double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

    drivingConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);   // 70 for SparkFlex
    // drivingConfig
    //   .encoder
    //     // .positionConversionFactor(drivingFactor) // meters
    //     .velocityConversionFactor(drivingFactor / 60.0); // meters per second
    drivingConfig
      .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // These are example gains you may need to adjust them for your own robot!
        .pid(0.04, 0, 0)
        .outputRange(-1, 1)
      .feedForward
        .kV(drivingVelocityFeedForward);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);   // 70 for SparkFlex
      turningConfig
        .absoluteEncoder
          // Do not invert the turning encoder, since the output shaft rotates in the same
          // direction as the steering motor in the EasySwerve Module.
          .inverted(false)
          // .positionConversionFactor(turningFactor) // radians
          // .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

      turningConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true);
          // .positionWrappingInputRange(0, turningFactor);

    pivotL
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(80)
      .voltageCompensation(12)
      .closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(0, 0, 0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false);
    pivotL.absoluteEncoder
      // .positionConversionFactor(360)
      .inverted(false)
      .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    pivotR
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(80)
      .voltageCompensation(12)
      .follow(9, false);

    elevatorL
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(80)
      .voltageCompensation(12)
      .closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .pid(0, 0, 0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false);

    elevatorR
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(60)
      .voltageCompensation(12)
      .follow(9, false);

    tilt
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(60)
      .voltageCompensation(12)
      .closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .pid(0, 0, 0)
      .outputRange(-1, 1)
      .positionWrappingEnabled(false);
    tilt.absoluteEncoder
      // .positionConversionFactor(360)
      .inverted(false)
      .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

    claw
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(40)
      .voltageCompensation(12);
  }
  
}
