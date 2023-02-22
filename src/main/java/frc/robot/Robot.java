// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */



public class Robot extends LoggedRobot {
  public final int kShoulderAnalogInputPort =  0;
  public final int kShoulderEncoderId =        24;
  public final int kElbowAnalogInputPort =     1;
  public final int kElbowEncoderId =           25;

  private final double kShoulderPotentiometerGearRatio           = 72.0/16.0;
  private final double kShoulderEncoderGearRatio                 = 72.0/16.0;
  private final double kShoulderPotentiometerNTurns              = 3.0;    

  private final double kElbowPotentiometerGearRatio              = 64.0/16.0;
  private final double kElbowEncoderGearRatio                    = 64.0/16.0;
  private final double kElbowPotentiometerNTurns                 = 3.0;


  AnalogPotentiometer shoulderPot = new AnalogPotentiometer(kShoulderAnalogInputPort, kShoulderPotentiometerNTurns*360.0);
  AnalogPotentiometer elbowPot = new AnalogPotentiometer(kElbowAnalogInputPort, kElbowPotentiometerNTurns*360.0);

  CANCoder shoulderEnc = new CANCoder(kShoulderEncoderId);
  CANCoder elbowEnc = new CANCoder(kElbowEncoderId);

  Logger logger = Logger.getInstance();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    CANCoderConfiguration canConfig = new CANCoderConfiguration();
    shoulderEnc.configAllSettings(canConfig);  // configure to default settings
    elbowEnc.configAllSettings(canConfig);  // configure to default settings

    // Record metadata
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    if (isReal()) {
      logger.addDataReceiver(new WPILOGWriter("/media/sda1/")); // Log to a USB stick
      logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
    // Logger.getInstance().disableDeterministicTimestamps()

    // Start AdvantageKit logger
    logger.start();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
  }

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  double shoulderPotAngleDeg = 0.0;
  double    elbowPotAngleDeg = 0.0;
  double shoulderAbsAngleDeg = 0.0;
  double    elbowAbsAngleDeg = 0.0;
  final double a = 1.0 / 100.0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // apply some averaging
    shoulderPotAngleDeg = (1-a)*shoulderPotAngleDeg + a*shoulderPot.get();
    shoulderAbsAngleDeg = (1-a)*shoulderAbsAngleDeg + a*shoulderEnc.getAbsolutePosition();
       elbowPotAngleDeg = (1-a)*elbowPotAngleDeg + a*elbowPot.get();
       elbowAbsAngleDeg = (1-a)*elbowAbsAngleDeg + a*elbowEnc.getAbsolutePosition();
    
    // record
    logger.recordOutput("Shoulder Pot (Deg)", shoulderPotAngleDeg);
    logger.recordOutput("Shoulder Abs (Deg)", shoulderAbsAngleDeg);
    logger.recordOutput("Elbow Pot (Deg)", elbowPotAngleDeg);
    logger.recordOutput("Elbow Abs (Deg)", elbowAbsAngleDeg);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
