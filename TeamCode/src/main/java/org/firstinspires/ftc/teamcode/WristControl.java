package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristControl {

    private Servo stdWristServo = null;

    public WristControl(HardwareMap hwMap, String slideLabel)
    {
        stdWristServo = hwMap.get(Servo.class, slideLabel);

    }

    public void turnUp()
    {
        stdWristServo.setPosition(StandardBot.WRIST_UP_POSITION);
    }

    public void turnDown()
    {
        stdWristServo.setPosition(StandardBot.WRIST_DOWN_POSITION);
    }

    public void turnToMiddle()
    {
        stdWristServo.setPosition(StandardBot.WRIST_MIDDLE_POSITION);
    }

    public void turnToRest()
    {
        stdWristServo.setPosition(StandardBot.WRIST_REST_POSITION);
    }
}
