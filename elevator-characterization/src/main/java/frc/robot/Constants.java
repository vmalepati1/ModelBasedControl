package frc.robot;

import static frc.robot.Constants.Elevator.*;

public class Constants {

    public static final class InputDevicePorts {

        public static final int GAMEPAD_PORT = 0;

    }

    public enum Robot {
        PRACTICE(0.7800, 0.000409, MINIMUM_HEIGHT, MAXIMUM_HEIGHT, 775, false, false),
        COMPETITION(0.7800, 0.0002045, MINIMUM_HEIGHT_COMP, MAXIMUM_HEIGHT_COMP, 386, true,
                true/*true*/);


        private final double robotWidth;
        private final double distancePerPulse;
        private final int minimumElevatorHeight;
        private final int maximumElevatorHeight;
        private final int inchesToNativeUnits;
        private final boolean isReversed;
        private final boolean sensorPhase;

        Robot(double robotWidth, double distancePerPulse, int minimumElevatorHeight,
              int maximumElevatorHeight, int inchesToNativeUnits, boolean isReversed,
              boolean SensorPhase) {

            this.robotWidth = robotWidth;
            this.distancePerPulse = distancePerPulse;
            this.minimumElevatorHeight = minimumElevatorHeight;
            this.maximumElevatorHeight = maximumElevatorHeight;
            this.inchesToNativeUnits = inchesToNativeUnits;
            this.isReversed = isReversed;
            sensorPhase = SensorPhase;
        }

        public int getInchesToNativeUnits() {
            return inchesToNativeUnits;
        }

        public int getMinimumElevatorHeight() {
            return minimumElevatorHeight;
        }

        public int getMaximumElevatorHeight() {
            return maximumElevatorHeight;
        }

        public double getRobotWidth() {
            return robotWidth;
        }

        public double getDistancePerPulse() {
            return distancePerPulse;
        }

        public boolean getSensorPhase() {
            return sensorPhase;
        }

        public boolean isReversed() {
            return isReversed;
        }
    }

    public static final class ElevatorChannels {

        public static final int ELEVATOR_MOTOR_CHANNEL = 3; // can
        public static final int ELEVATOR_LIMIT_LOWER_CHANNEL = 4; // digital

    }

    public static final class Elevator {

        public static final double TOLERANCE = 0.1;

        public static final double HIGH_HEIGHT = 68; // inches, this gets the scale
        public static final double MEDIUM_HEIGHT = 24.00; // inches, this gets the switch, exchange top, and portal
        public static final double LOW_HEIGHT = 0; // inches, the floor

        public static final int MINIMUM_HEIGHT = 1000; // in nu (native units)
        public static final int MAXIMUM_HEIGHT = 50900; // in nu

        public static final int MINIMUM_HEIGHT_COMP = 00; // 1000 in nu (native units)
        public static final int MAXIMUM_HEIGHT_COMP = 27200; //2700 -  // in nu

        public static final double NUDGE_DISTANCE = 1; // in inches

        public static final int CRUISE_SPEED = 2500; // native sensor units per 100 ms
        public static final int ACCELERATION = 2000; // ^^^ per second

        // motion control constants
        public static final double KF = 0.4661;
        public static final double KP = 1;
        public static final double KI = 0.001;
        public static final double KD = 0;

        public static final int TIMEOUT = 100; // in ms

        private Elevator() {
        }
    }

}
