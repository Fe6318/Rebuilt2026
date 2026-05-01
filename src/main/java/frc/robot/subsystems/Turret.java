// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;


public class Turret extends SubsystemBase {
  private SparkMax turretMotor = new SparkMax(12, MotorType.kBrushless);
  
SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 4, 9.5)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.LOW)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(40))
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25));

SmartMotorController motor = new SparkWrapper(turretMotor,DCMotor.getNEO(1),motorConfig);

PivotConfig                turretConfig         = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
     
      .withHardLimit(Degrees.of(-90), Degrees.of(90)) // Hard limit bc wiring prevents infinite spinning
      .withTelemetry("Turret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

      private Pivot turret = new Pivot(turretConfig);  
  /** Creates a new Turret. */
  public Turret() {}
   /**
   * Set the angle of the arm.
   * @param angle Angle to go to.
   */
  public Command setAngle(Angle angle) { return turret.setAngle(angle); }
  /**
   * Set the dutycycle of the turret.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return turret.set(dutyCycle);}
  public Command zero() {return runOnce(() -> { turretMotor.getEncoder().setPosition(0); });}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turret.updateTelemetry();
  }
   @Override
  public void simulationPeriodic(){
    
    // This method will be called once per scheduler run during simulation
    turret.simIterate();
  }
}
