package frc.robot;

public final class Constants {
    public static final class CANMapping {

        // DriveTrain 2023 Bot
        public static final int SPARKMAX_DRIVE_BL = 52;
        public static final int SPARKMAX_DRIVE_BR = 53;
        public static final int SPARKMAX_DRIVE_FL = 54;
        public static final int SPARKMAX_DRIVE_FR = 51;

        public static final int TALONSRX_TURN_BL = 32;
        public static final int TALONSRX_TURN_BR = 33;
        public static final int TALONSRX_TURN_FL = 34;
        public static final int TALONSRX_TURN_FR = 31;

        public static final int TURN_CANCODER_BL = 42;
        public static final int TURN_CANCODER_BR = 43;
        public static final int TURN_CANCODER_FL = 44;
        public static final int TURN_CANCODER_FR = 41;
        
        // MISC CAN Bus
        public static final int PIGEON_IMU = 20;
        //public static final int PNEUMATIC_HUB = 5;
    }

    public static final class ControllerMapping {
        public static final int JOYSTICK = 0;
        public static final int XBOX = 1;
    }

    public static final class PneumaticsMapping {
        // Pneumatics Control Module
        public static final int PNEUMATIC_SINGLE_SOLENOID_EXTEND = 1;
        public static final int PNEUMATIC_SINGLE_SOLENOID_RETRACT = 0;
    }

    public static final class MiscMapping {
        public static final boolean BRAKE_ON = true;
        public static final boolean BRAKE_OFF = false;
        public static final boolean FIELD_RELATIVE = false;
    }
}
