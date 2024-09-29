package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot { // H Drive底盤搭配小型手臂
  private final TalonFX lf = new TalonFX(1);
  private final TalonFX lb = new TalonFX(2);
  private final TalonFX rf = new TalonFX(3);
  private final TalonFX rb = new TalonFX(0);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  AHRS m_gyro = new AHRS(Port.kMXP);

  // limit switch
  private final DigitalInput toplimit = new DigitalInput(1);
  private final DigitalInput bottomlimit = new DigitalInput(0);
  private final TalonSRX hand = new TalonSRX(8);

  double speed = 0.4;

  double kp = 0.012;
  double ki = 0.0336;
  double kd = 0.001575;

  double tp = 0.0186;
  double ti = 0.0248;
  double td = 0.01;

  double error, errorsum, iLimit = 0.25, output, errorate = 0, last = 0, lasterror = 0;
  double angleerror, anglesum = 0, anglerate, lastangle = 0, ilimit = 11.8;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_gyro.calibrate();
    m_gyro.reset();

    lf.set(ControlMode.PercentOutput, 0);
    lb.set(ControlMode.PercentOutput, 0);
    rf.set(ControlMode.PercentOutput, 0);
    rb.set(ControlMode.PercentOutput, 0);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_gyro.reset();
  }

  @Override
  public void teleopPeriodic() {
    if (m_stick.getRawButton(6)) {
      m_gyro.reset();
    }

    boolean right = true;
    boolean left = true;

    double current = Timer.getFPGATimestamp();
    error = m_gyro.getAngle();
    m_timer.reset();

    if (Math.abs(m_stick.getRawAxis(5)) > 0.1 && Math.abs(m_stick.getRawAxis(4)) < 0.1) {
      right = false;
      left = false;
      double dt = current - last;
      if (Math.abs(error) < iLimit) {
        errorsum += error * dt;
      }
      errorate = (error - lasterror) / dt;
      output = kp * error + ki * errorsum + kd * errorate;
      lf.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * -0.15 - output);
      lb.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * -0.15 - output);
      rf.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.15 - output);
      rb.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.15 - output);
    }
    last = current;
    lasterror = error;

    if (Math.abs(m_stick.getRawAxis(5)) < 0.1 && Math.abs(m_stick.getRawAxis(4)) > 0.1) {
      if (m_stick.getRawAxis(4) > 0.1) {
        left = false;
        right = true;
      } else {
        left = true;
        right = false;
      }
      lf.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.08);
      lb.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.08);
      rf.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.08);
      rb.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.08);
    }

    if (Math.abs(m_stick.getRawAxis(5)) < 0.05 && Math.abs(m_stick.getRawAxis(4)) < 0.05 && right == true
        && left == true) {
      lf.set(ControlMode.PercentOutput, 0);
      lb.set(ControlMode.PercentOutput, 0);
      rf.set(ControlMode.PercentOutput, 0);
      rb.set(ControlMode.PercentOutput, 0);
      m_gyro.reset();
    }

    error = m_gyro.getYaw();

    if (m_stick.getRawAxis(3) > 0.1) {
      speed += m_stick.getRawAxis(3) * 0.01;
    } else if (m_stick.getRawAxis(3) < 0.1) {
      speed = 0.4;
    }

    angleerror = 90 - Math.abs(error);

    if (m_stick.getRawButton(1)) {
      double dt = current - last;

      if (Math.abs(error) < iLimit) {
        errorsum += error * dt;
      }
      errorate = (error - lasterror) / dt;
      output = kp * error + ki * errorsum + kd * errorate;

      m_robotDrive.tankDrive(speed - output, speed + output);

      last = current;
      lasterror = error;
    }

    if (m_stick.getRawButton(4)) {
      double dt = current - last;
      if (Math.abs(error) < iLimit) {
        errorsum += error * dt;
      }
      errorate = (error - lasterror) / dt;
      output = kp * error + ki * errorsum + kd * errorate;

      m_robotDrive.tankDrive(-speed - output, -speed + output);

      last = current;
      lasterror = error;
    }

    if (m_stick.getRawButton(2)) {
      double dt = current - last;
      if (Math.abs(angleerror) < ilimit) {
        anglesum += angleerror * dt;
      }
      anglerate = (angleerror - lastangle) / dt;

      if (m_gyro.getYaw() < 80) {
        m_robotDrive.arcadeDrive(0, 0.4);
      } else if (m_gyro.getYaw() > 80 && m_gyro.getYaw() != 90) {
        output = angleerror * tp + anglesum * ti + anglerate * td;
        m_robotDrive.arcadeDrive(0, output * 0.4);
      } else {
        m_robotDrive.stopMotor();
      }
      last = current;
      lastangle = angleerror;
    }

    if (m_stick.getRawButton(3)) {
      double dt = current - last;
      if (Math.abs(angleerror) < ilimit) {
        anglesum += angleerror * dt;
      }
      anglerate = (angleerror - lastangle) / dt;

      if (Math.abs(m_gyro.getYaw()) < 80) {
        m_robotDrive.arcadeDrive(0, -0.4);
      } else if (Math.abs(m_gyro.getYaw()) > 80 && Math.abs(m_gyro.getYaw()) != 90) {
        output = angleerror * tp + anglesum * ti + anglerate * td;
        m_robotDrive.arcadeDrive(0, -output * 0.4);
      } else {
        m_robotDrive.stopMotor();
      }
      last = current;
      lastangle = angleerror;

    }
    SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("angleerror", angleerror);

    SmartDashboard.putNumber("angle", m_gyro.getAngle());
    SmartDashboard.putNumber("error", error);
    SmartDashboard.putNumber("output", output);

    setMotorSpeed(m_stick.getRawAxis(1) * -0.3);
  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (toplimit.get()) {
        hand.set(ControlMode.PercentOutput, 0);
      } else {
        hand.set(ControlMode.PercentOutput, speed);
      }
    } else {
      if (bottomlimit.get()) {
        hand.set(ControlMode.PercentOutput, 0);
      } else {
        hand.set(ControlMode.PercentOutput, speed);
      }
    }
    if (speed == 0) {
      hand.set(0);
    }
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}
