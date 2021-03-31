// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.LineSensor;
import frc.robot.subsystems.RomiDrivetrain;

public class LineFollowerCommand extends CommandBase {

	private final RomiDrivetrain m_drivetrain;

	private final LineSensor m_leftSensor;
	private final LineSensor m_middleSensor;
	private final LineSensor m_rightSensor;

	private static final int kThreshold = 800;
	private static final double kSpeedVolts = 1.5;

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
		if (m_middleSensor.check()) m_drivetrain.tankDriveVolts(kSpeedVolts, kSpeedVolts);
		else if (m_rightSensor.check()) m_drivetrain.tankDriveVolts(-kSpeedVolts, kSpeedVolts);
		else if (m_leftSensor.check()) m_drivetrain.tankDriveVolts(kSpeedVolts, -kSpeedVolts);
		else m_drivetrain.tankDriveVolts(kSpeedVolts, kSpeedVolts);
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
