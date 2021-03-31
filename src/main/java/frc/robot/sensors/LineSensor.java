// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class LineSensor {
	private AnalogInput m_input;

	private int m_BGLevel = 0;
	private int m_threshold = -1;

	public LineSensor(int port) {
		m_input = new AnalogInput(port);
	}

	public int read() {
		return m_input.getValue();
	}

	public int calibratedRead() {
		return read() - m_BGLevel;
	}

	public boolean check() {
		return calibratedRead() > m_threshold;
	}

	public void calibrate() {
		m_BGLevel = 400;
	}

	public void setThreshold(int threshold) {
		m_threshold = threshold;
	}

	public boolean calStatus() {
		return m_BGLevel != -1 && m_threshold != -1;
	}

}
