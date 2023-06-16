package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GripperControl {

    private Servo stdGripperServo = null;

    public GripperControl(HardwareMap hwMap, String slideLabel)
    {
        stdGripperServo = hwMap.get(Servo.class, slideLabel);

    }

    public void open()
    {
        stdGripperServo.setPosition(StandardBot.GRIPPER_OPENED_POSITION);
    }

    public void close()
    {
        stdGripperServo.setPosition(StandardBot.GRIPPER_CLOSED_POSITION);
    }
}
