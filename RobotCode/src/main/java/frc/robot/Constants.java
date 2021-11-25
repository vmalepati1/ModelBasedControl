package frc.robot;

public class Constants {

    public static final class InputDevices {

        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;

    }

    public static final class CANBusIDs {

        public static final int DRIVETRAIN_LEFT_MASTER_ID = 3;
        public static final int DRIVETRAIN_LEFT_SLAVE_ID = 4;

        public static final int DRIVETRAIN_RIGHT_MASTER_ID = 1;
        public static final int DRIVETRAIN_RIGHT_SLAVE_ID = 2;

    }

    public static final class AnalogChannels {

        public static final int GYRO_CHANNEL = 1;

    }

    public static final class DrivetrainConstants {

        public static final int K_SMART_CURRENT_LIMIT = 40;
        public static final int K_OPENLOOP_RAMP = 0;

        // Create our feedforward gain constants (from the characterization tool)
        public static final double KsLinear = 0.216;
        public static final double KvLinear = 4.58;
        public static final double KaLinear = 0.322;
        public static final double KvAngular = 4.51;
        public static final double KaAngular = 0.22;

        public static final double POSITION_FACTOR = 1.0 / 36.797;
        public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.;

        public static final double VOLTAGE_P = 1.0;

        public static final double LEFT_VELOCITY_P = 0.35;
        public static final double RIGHT_VELOCITY_P = 0.35;

        public static final double TRACK_WIDTH = 0.8285293767383018;

    }

}
