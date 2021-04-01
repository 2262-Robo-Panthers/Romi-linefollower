// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.LineSensor;
import frc.robot.sensors.RomiGyro;

public class RomiDrivetrain extends SubsystemBase {
	private static final double kCountsPerRevolution = 1440.0;
	private static final double kWheelDiameterMeter = 0.07; // 70 mm

	// The Romi has the left and right motors set to
	// PWM channels 0 and 1 respectively
	private final Spark m_leftMotor = new Spark(0);
	private final Spark m_rightMotor = new Spark(1);

	// The Romi has onboard encoders that are hardcoded
	// to use DIO pins 4/5 and 6/7 for the left and right
	private final Encoder m_leftEncoder = new Encoder(4, 5);
	private final Encoder m_rightEncoder = new Encoder(6, 7);

	@SuppressWarnings("unused")
	private final RomiGyro m_gyro = new RomiGyro();

	@SuppressWarnings("unused")
	private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer(Range.k2G);

	private final LineSensor m_leftSensor = new LineSensor(2);
	private final LineSensor m_middleSensor = new LineSensor(1);
	private final LineSensor m_rightSensor = new LineSensor(3);

	// Set up the differential drive controller
	private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

	/** Creates a new RomiDrivetrain. */
	public RomiDrivetrain() {
		// Use meters as unit for encoder distances
		m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
		resetEncoders();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("Left Sensor Raw", m_leftSensor.read());
		SmartDashboard.putNumber("Middle Sensor Raw", m_middleSensor.read());
		SmartDashboard.putNumber("Right Sensor Raw", m_rightSensor.read());
		SmartDashboard.putNumber("Left Motor", m_leftMotor.get());
		SmartDashboard.putNumber("Right Motor", m_rightMotor.get());
	}

	public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
		m_diffDrive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	public void stopDrivetrain() {
		m_diffDrive.stopMotor();
	}

	public void tankDriveVolts(double left, double right) {
		m_leftMotor.setVoltage(left);
		m_rightMotor.setVoltage(-right);
		m_diffDrive.feed();
	}

	public LineSensor getLeftSensor() {
		return m_leftSensor;
	}

	public LineSensor getMiddleSensor() {
		return m_middleSensor;
	}

	public LineSensor getRightSensor() {
		return m_rightSensor;
	}

}
