// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.hal;

public class LEDJNI extends JNIWrapper {
  public static final int RADIO_LED_STATE_OFF = 0;
  public static final int RADIO_LED_STATE_GREEN = 1;
  public static final int RADIO_LED_STATE_RED = 2;
  public static final int RADIO_LED_STATE_ORANGE = 3;

  /**
   * Set the state of the "Radio" LED.
   *
   * @param state The state to set the LED to.
   * @see "HAL_SetRadioLEDState"
   */
  public static native void setRadioLEDState(int state);

  /**
   * Get the state of the "Radio" LED.
   *
   * @return The state of the LED.
   * @see "HAL_GetRadioLEDState"
   */
  public static native int getRadioLEDState();

  /** Utility class. */
  private LEDJNI() {}
}
