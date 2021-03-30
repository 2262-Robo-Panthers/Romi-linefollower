// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class LineSensor {
	private AnalogInput m_input;

	private int m_BGLevel = -1;
	private int m_detectLevel = -1;

	public LineSensor(int port) {
		m_input = new AnalogInput(port);
	}

	public int read() {
		return m_input.getValue();
	}

	public boolean check() {
		final int level = read();
		if (m_BGLevel < m_detectLevel) {
			final int threshold = (m_detectLevel - m_BGLevel) >> 2;
			return level - threshold > m_BGLevel;
		} else {
			final int threshold = (m_BGLevel - m_detectLevel) >> 2;
			return level + threshold < m_BGLevel;
		}
	}

	public int setBGLevel() {
		m_BGLevel = read();
		return m_BGLevel;
	}

	public int setDetectLevel() {
		m_detectLevel = read();
		return m_detectLevel;
	}

	public boolean calStatus() {
		return m_BGLevel != -1 && m_detectLevel != -1;
	}

}
