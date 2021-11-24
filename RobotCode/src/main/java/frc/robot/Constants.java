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

    }

}
