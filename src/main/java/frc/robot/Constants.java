// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double kDefaultArmSetpointDegrees = 180.0; 

  public static final double kDefaultArmKp = 1.0; 

  public static final String kArmPositionKey = "ArmPosition";

  public static final String kArmPKey = "ArmP"; 

  //public static final int kDriverControllerPort = 0;
    
  //public static final int kMotorID = 2;
  
  //public static final int kContinuousCurrentLimit = 20;
  //public static final int kPeakCurrentLimit = 50;
  //public static final double kPeakCurrentDuration = 0.5;

  // public static final double kP = 2;
  // public static final double kI = 0;
  // public static final double kD = 0.07;
  // public static final double kF = 0;

  // public static final boolean kUseTelemetry = false; 
  // /** the gravity compensation amount as a percent of full motor power, scaled by a cosine term (this value is the max FF) */
  // public static final double kGravityCompensation = 0.02;

  // public static final double kTolerance = 0.02;
  // public static final double kMotorPowerClamp = 0.8;

  //public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.CounterClockwise;

  public static final NeutralMode kNeutralMode = NeutralMode.Brake;

  /** RoboRIO digital input port for the wrist absolute encoder */
  //public static final int kAbsEncoderPort = 7;
  /** wrist absolute encoder offset (rotations). */
  //public static final double kEncoderOffset = 0.687; // Black tread: 0.687, blue tread: 0.704

  //public static final double kAutoTop = Units.rotationsToRadians(0.070);
  //public static final double kAutoMiddle = Units.rotationsToRadians(0.132);

  // public static final double kStowPos = 1.95;
  // public static final double kBottomNodeCubePos = Units.rotationsToRadians(0.170);
  // public static final double kMiddleNodeCubePos = 0.551; //Units.rotationsToRadians(0.170);
  // public static final double kTopNodeCubePos = 1.022;

  // public static final double kBottomNodeConePos = Units.rotationsToRadians(0.070);
  // public static final double kMiddleNodeConePos = Units.rotationsToRadians(0.070);
  // public static final double kTopNodeConePos = Units.rotationsToRadians(0.070);
  
  // public static final double kIntakeConePos = 0.25; //blue tread: 0.1571; // 0.2199
  // public static final double kIntakeCubePos = 0.1;
  // public static final double kIntakeShelfPos = Units.rotationsToRadians(0.04);

  //public static final double kIntakeAltShelfPos = 1.35;

  /** Wrist position angle minimum (radians) */
  public static final double kMinPos = 0;
  /** Wrist position angle maximum (radians) */
  public static final double kMaxPos = 6.28;

  //SIM
  // to know how much the arm will move with a certain power, the sim needs to know the motor, gear ratio, MOI, and length
  public static final DCMotor kGearBox = DCMotor.getFalcon500(1);
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kGearRatio = (20.0/1.0) * (62.0/34.0) * (48.0/18.0);
  
  //calculate MOI using the center of gravity distance and weight
  public static double kCOGWeight = Units.lbsToKilograms(7.3);
  public static double kCOGDistance = Units.inchesToMeters(8.11);
  
  /** Wrist moment of inertia represents how hard it is to angularly accelerate (ie spin) something. */
  public static final double kMomentOfInertia = kCOGWeight * kCOGDistance * kCOGDistance; // 0.1405


  /** Length of the wrist joint (meters). Used in simulation and estimating accuracy. */
  public static final double kLength = Units.inchesToMeters(16.1);
}


