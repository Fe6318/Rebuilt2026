// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import yams.gearing.GearBox;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivot extends SubsystemBase {
  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(10, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  // https://www.reca.lc/arm?armMass=%7B%22s%22%3A0.5%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A6%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=100&endAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A12%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-90%2C%22u%22%3A%22deg%22%7D
  .withFeedforward(new ArmFeedforward(0, 0, .0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("PivotMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));
   // Vendor motor controller object
  private SparkMax spark = new SparkMax(25, MotorType.kBrushless);
  public IntakePivot() {}
  // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private ArmConfig armCfg = new ArmConfig(sparkSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-75), Degrees.of(75))
  // Hard limit is applied to the simulation.
  .withHardLimit(Degrees.of(-90), Degrees.of(90))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(0))
  // Length and mass of your arm for sim.
  .withLength(Feet.of(1))
  .withMass(Pounds.of(1))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH);

  // Arm Mechanism
  private Arm arm = new Arm(armCfg);
  /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) { return arm.setAngle(angle);}

  /**
   * Move the arm up and down.
   * @param dutycycle [-1, 1] speed to set the arm too.
   */
  public Command set(double dutycycle) { return arm.set(dutycycle);}

  /**
   * Run sysId on the {@link Arm}
   */
  public Command sysId() { return arm.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arm.updateTelemetry();
  }
   @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    arm.simIterate();
  }
}
