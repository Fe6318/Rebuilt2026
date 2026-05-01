// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.units.measure.AngularVelocity;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

  public class Feeder extends SubsystemBase {

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(0, 0, 0)
  .withSimClosedLoopController(0, 0, 0)
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("Feeder", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
  // You could also use .withGearing(12) which does the same thing.
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(4)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(true)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(20));

   // Vendor motor controller object
  private SparkMax spark = new SparkMax(32, MotorType.kBrushless);
    // Create our SmartMotorController from our Spark and config with the NEO.
  private SmartMotorController sparkSmartMotorController = new SparkWrapper(spark, DCMotor.getNeo550(1), smcConfig);
  /** Creates a new ballElevator. */
 private final FlyWheelConfig feederConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel.
  .withDiameter(Inches.of(4))
  // Mass of the flywheel.
  .withMass(Pounds.of(1))
  // Maximum speed of the ballElevator.
  .withUpperSoftLimit(RPM.of(10000))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("Feeder", TelemetryVerbosity.LOW);

  // Shooter Mechanism
  private FlyWheel feeder = new FlyWheel(feederConfig);
    /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return feeder.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return feeder.run(speed);}
  
  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {feeder.setMechanismVelocitySetpoint(speed);}

    /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return feeder.set(dutyCycle);}
  
  public Feeder() {}

  @Override
  public void periodic() {
     // This method will be called once per scheduler run
    feeder.updateTelemetry();
  }

    @Override
  public void simulationPeriodic(){
    
    // This method will be called once per scheduler run during simulation
    feeder.simIterate();
  }
}
