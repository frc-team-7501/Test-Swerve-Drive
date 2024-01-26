// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ExtendedJoystick extends Joystick {
    /**
     * Represents the buttons on a Logitech Extreme 3D joystick controller.
     */
    public static enum Button {
        kTrigger(1),
        kSide(2),
        kTopBL(3),
        kTopBR(4),
        kTopTL(5),
        kTopTR(6),
        kBottomFL(7),
        kBottomFR(8),
        kBottomCL(9),
        kBottomCR(10),
        kBottomBL(11),
        kBottomBR(12);

        @SuppressWarnings("MemberName")
        public final int value;

        Button(int value) {
        this.value = value;
        }
    }

    private JoystickButton k_b_Trigger = getButton(Button.kTrigger);
    private JoystickButton k_b_Side = getButton(Button.kSide);
    private JoystickButton k_b_TopBL = getButton(Button.kTopBL);
    private JoystickButton k_b_TopBR = getButton(Button.kTopBR);
    private JoystickButton k_b_TopTL = getButton(Button.kTopTL);
    private JoystickButton k_b_TopTR = getButton(Button.kTopTR);
    private JoystickButton k_b_BottomFL = getButton(Button.kBottomFL);
    private JoystickButton k_b_BottomFR = getButton(Button.kBottomFR);
    private JoystickButton k_b_BottomCL = getButton(Button.kBottomCL);
    private JoystickButton k_b_BottomCR = getButton(Button.kBottomCR);
    private JoystickButton k_b_BottomBL = getButton(Button.kBottomBL);
    private JoystickButton k_b_BottomBR = getButton(Button.kBottomBR);

    public ExtendedJoystick(int port) {
        super(port);
    }

    public JoystickButton getButton(int button) {
        return new JoystickButton(this, button);
    }

    public JoystickButton getButton(Button button) {
        return new JoystickButton(this, button.value);
    }

    public JoystickButton b_Trigger() { return k_b_Trigger; }
    public JoystickButton b_Side() { return k_b_Side; }
    public JoystickButton b_TopBL() { return k_b_TopBL; }
    public JoystickButton b_TopBR() { return k_b_TopBR; }
    public JoystickButton b_TopTL() { return k_b_TopTL; }
    public JoystickButton b_TopTR() { return k_b_TopTR; }
    public JoystickButton b_BottomFL() { return k_b_BottomFL; }
    public JoystickButton b_BottomFR() { return k_b_BottomFR; }
    public JoystickButton b_BottomCL() { return k_b_BottomCL; }
    public JoystickButton b_BottomCR() { return k_b_BottomCR; }
    public JoystickButton b_BottomBL() { return k_b_BottomBL; }
    public JoystickButton b_BottomBR() { return k_b_BottomBR; }
}
