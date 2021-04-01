// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.LineSensor;
import frc.robot.subsystems.RomiDrivetrain;

public class LineFollowerCommand extends CommandBase {

	private final RomiDrivetrain m_drivetrain;

	private final LineSensor m_leftSensor;
	private final LineSensor m_middleSensor;
	private final LineSensor m_rightSensor;

	private static final int kThreshold = 800;
	private static final double kSpeedVolts = 6;
	private static final double kI = 0.05;
	private static final double kF = 0.5;

	private double m_integrator = 0;
	private boolean m_isLeft = true;
	private double m_lastTimestamp = -1;

	/** Creates a new LineFollowerCommand. */
	public LineFollowerCommand(RomiDrivetrain drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		m_drivetrain = drivetrain;
		m_leftSensor = drivetrain.getLeftSensor();
		m_middleSensor = drivetrain.getMiddleSensor();
		m_rightSensor = drivetrain.getRightSensor();
		addRequirements(drivetrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_leftSensor.setThreshold(kThreshold);
		m_middleSensor.setThreshold(kThreshold);
		m_rightSensor.setThreshold(kThreshold);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_rightSensor.check()) {
			m_drivetrain.tankDriveVolts(kSpeedVolts * (0.5 + kF), kSpeedVolts * (0.5 - kF));
			m_isLeft = false;
		}
		else if (m_leftSensor.check()) {
			m_drivetrain.tankDriveVolts(kSpeedVolts * (0.5 - kF), kSpeedVolts * (0.5 + kF));
			m_isLeft = true;
		}
		else {
			m_drivetrain.tankDriveVolts(kSpeedVolts, kSpeedVolts);
		}
		double dt = m_lastTimestamp > 0 ? Timer.getFPGATimestamp() - m_lastTimestamp : 0.02;
		m_lastTimestamp = Timer.getFPGATimestamp();
		m_integrator += m_middleSensor.check() ? 0 : kI * (m_isLeft ? 1.0 : -1.0) * dt / 0.02;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_drivetrain.stopDrivetrain();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_leftSensor.check() && m_middleSensor.check() && m_rightSensor.check();
	}
}
