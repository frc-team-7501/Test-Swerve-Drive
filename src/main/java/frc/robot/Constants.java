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

        // Other Motors
        public static final int LAUNCH_TALON = 12;
        public static final int HANDOFF_SPARKMAX = 14;
        public static final int INTAKE_TALON = 11;

        // MISC CAN Bus
        public static final int PIGEON_IMU = 20;
        // public static final int PNEUMATIC_HUB = 5;
    }

    public static final class ControllerMapping {
        public static final int JOYSTICK = 1;
        public static final int XBOX = 0;
    }

    public static final class PneumaticsMapping {
        // Pneumatics Control Module
        public static final int PNEUMATIC_SINGLE_SOLENOID_EXTEND = 1;
        public static final int PNEUMATIC_SINGLE_SOLENOID_RETRACT = 0;
    }

    public static final class MiscMapping {
        public static final boolean BRAKE_ON = true;
        public static final boolean BRAKE_OFF = false;
        public static final boolean FIELD_RELATIVE = true;
        public static final double MAXSPEED = 0.7;
        public static final double MAXANGULARSPEED = 0.8;
        public static final double LAUNCH_VELOCITY = 40;
        public static final double INTAKE_VELOCITY = -25;
        public static final double HANDOFF_SPEED = -0.3;

    }

    public static final class TalonMapping {
        public static final double PID_P = 0.11;// An error of 1 rotation per second results in 2V output
        public static final double PID_I = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        public static final double PID_D = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        public static final double PID_V = 0.12; //Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        public static final double PEAK_VOLTAGE = 11; // Peak output of X volts on a Falcon.
        public static final double PEAK_AMPERAGE = 80; // Peak output of X amps on a Falcon.
    }
}
