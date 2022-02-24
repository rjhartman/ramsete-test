package frc.robot.motorcontrol;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants.DriveConstants;

/**
 * A wrapper class for the TalonFX motor controller to give
 * helpful functions like getRate and getDistance for the
 * integrated encoder.
 */
public class Luna_TalonFX extends WPI_TalonFX {

    /**
     * A default constructor needed to compile. DO NOT USE THIS CONSTRUCTOR.
     * A TalonFX cannot be instantiated without parameters.
     */
    public Luna_TalonFX() {
        super(-1);
        throw new IllegalStateException("Cannot construct Luna_TalonFX, no arguments given to constructor.");
    }

    /**
     * Constructs a Luna_TalonFX for a CAN device number.
     * 
     * @param deviceNumber The TalonFX's CAN device number [0,62]
     */
    public Luna_TalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Get the selected sensor velocity.
     * 
     * @return the selected sensor's velocity in meters per second.
     */
    public double getRate() {
        return getSelectedSensorVelocity() * DriveConstants.ENCODER_DISTANCE_PER_PULSE * 10;
    }

    /**
     * Get the selected sensor distance.
     * 
     * @return the selected sensor's distance in meters.
     */
    public double getDistance() {
        return getSelectedSensorPosition() * DriveConstants.ENCODER_DISTANCE_PER_PULSE * 10;
    }

    /**
     * Reset the internal encoder.
     */
    public void reset() {
        setSelectedSensorPosition(0.0);
    }
}
