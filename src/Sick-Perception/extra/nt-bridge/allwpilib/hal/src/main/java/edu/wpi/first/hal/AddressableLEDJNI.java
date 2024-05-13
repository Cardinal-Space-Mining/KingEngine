// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.hal;

/**
 * Addressable LED HAL JNI Methods.
 *
 * @see "hal/AdressableLED.h"
 */
public class AddressableLEDJNI extends JNIWrapper {
  /**
   * Initialize Addressable LED using a PWM Digital handle.
   *
   * @param pwmHandle handle of the digital port for PWM
   * @return Addressable LED handle
   * @see "HAL_InitializeAddressableLED"
   */
  public static native int initialize(int pwmHandle);

  /**
   * Free the Addressable LED Handle.
   *
   * @param handle the Addressable LED handle to free
   * @see "HAL_FreeAddressableLED"
   */
  public static native void free(int handle);

  /**
   * Sets the length of the LED strip.
   *
   * <p>The max length is 5460 LEDs.
   *
   * @param handle the Addressable LED handle
   * @param length the strip length
   * @see "HAL_SetAddressableLEDLength"
   */
  public static native void setLength(int handle, int length);

  /**
   * Sets the led output data.
   *
   * <p>If the output is enabled, this will start writing the next data cycle. It is safe to call,
   * even while output is enabled.
   *
   * @param handle the Addressable LED handle
   * @param data the buffer to write
   * @see "HAL_WriteAddressableLEDData"
   */
  public static native void setData(int handle, byte[] data);

  /**
   * Sets the bit timing.
   *
   * <p>By default, the driver is set up to drive WS2812Bs, so nothing needs to be set for those.
   *
   * @param handle the Addressable LED handle
   * @param highTime0NanoSeconds high time for 0 bit (default 400ns)
   * @param lowTime0NanoSeconds low time for 0 bit (default 900ns)
   * @param highTime1NanoSeconds high time for 1 bit (default 900ns)
   * @param lowTime1NanoSeconds low time for 1 bit (default 600ns)
   * @see "HAL_SetAddressableLEDBitTiming"
   */
  public static native void setBitTiming(
      int handle,
      int highTime0NanoSeconds,
      int lowTime0NanoSeconds,
      int highTime1NanoSeconds,
      int lowTime1NanoSeconds);

  /**
   * Sets the sync time.
   *
   * <p>The sync time is the time to hold output so LEDs enable. Default set for WS2812B.
   *
   * @param handle the Addressable LED handle
   * @param syncTimeMicroSeconds the sync time (default 280us)
   * @see "HAL_SetAddressableLEDSyncTime"
   */
  public static native void setSyncTime(int handle, int syncTimeMicroSeconds);

  /**
   * Starts the output.
   *
   * <p>The output writes continuously.
   *
   * @param handle the Addressable LED handle
   * @see "HAL_StartAddressableLEDOutput"
   */
  public static native void start(int handle);

  /**
   * Stops the output.
   *
   * @param handle the Addressable LED handle
   * @see "HAL_StopAddressableLEDOutput"
   */
  public static native void stop(int handle);

  /** Utility class. */
  private AddressableLEDJNI() {}
}
