package frc.robot.commands.customVision;

import edu.wpi.first.math.util.Units;

/**
 * Manufacturer specifications for the SWYFT USB camera (SR-CAMERA-OV9281-USBC-01). Pure data holder
 * -- no behavior. Import statically if you want bare names, e.g. {@code import static
 * frc.robot.commands.customVision.SwyftUsbCameraConstants.*;}
 */
public final class SwyftUsbCameraConstants {
  private SwyftUsbCameraConstants() {} // prevent instantiation

  /** Sensor: OV9281-class global-shutter monochrome. */
  public static final String SENSOR_DESCRIPTION = "OV9281-class global-shutter monochrome";

  /** Native/maximum resolution reported by the sensor (WXGA). */
  public static final int NATIVE_WIDTH_PIXELS = 1280;

  public static final int NATIVE_HEIGHT_PIXELS = 800;

  /** Published field of view at the native resolution, in degrees. */
  public static final double HORIZONTAL_FOV_DEGREES = 81.0;

  public static final double VERTICAL_FOV_DEGREES = 52.0;

  /** Maximum frame rate at the native 1280x800 resolution; up to 210 FPS at lower resolutions. */
  public static final double MAX_FPS_AT_NATIVE_RESOLUTION = 120.0;

  /** Lens is fixed-focus, factory glued at this focal distance -- not adjustable at runtime. */
  public static final double FACTORY_FOCUS_DISTANCE_METERS = Units.feetToMeters(5.0);

  /**
   * Factory ChArUco calibration at the native 1280x800 resolution, as published by SWYFT for this
   * SKU. The intrinsics matrix is {@code [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]}.
   */
  public static final double FACTORY_FX_AT_NATIVE_RESOLUTION = 799.4949524650956;

  public static final double FACTORY_FY_AT_NATIVE_RESOLUTION = 799.8320596652597;
  public static final double FACTORY_CX_AT_NATIVE_RESOLUTION = 644.0110163765805;
  public static final double FACTORY_CY_AT_NATIVE_RESOLUTION = 447.8047664027805;

  /**
   * Factory distortion coefficients {@code [k1, k2, p1, p2, k3]} (rectilinear/plumb-bob model), at
   * the native 1280x800 resolution. Distortion coefficients are
   */
}
