package org.firstinspires.ftc.teamcode.subsystems.dunk;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.hang.linearActuator;


public class servoArmDunk implements Dunk {

    private DcMotor hanger;
    private Servo dunker;

    //Dunk Servo Variables
    private final double DUNK_POSITION = 0.23;
    private final double DOWN_POSTITION = 0.84;
    private final double WIGGLE_POSITION1 = 0.23;
    private final double WIGGLE_POSITION2 = 0.2;

    public servoArmDunk(DcMotor verticalMotion, Servo dunkArm)
    {
        hanger = verticalMotion;
        dunker = dunkArm;
    }

    //Dunk
    @Override
    public void dunkWithPause() throws InterruptedException
    {
        dunker.setPosition(DUNK_POSITION+0.22);

        Thread.sleep(1300);

        dunker.setPosition(DUNK_POSITION);
    }

    //Dunk return
    @Override
    public void down() throws InterruptedException
    {
        dunker.setPosition(DOWN_POSTITION);
    }

    //Dunk
    @Override
    public void dunkNoPause() throws InterruptedException
    {
        dunker.setPosition(DUNK_POSITION);
    }

    @Override
    public void wiggle()
    {
        for (int n = 0; n <= 3; n++)
        {
            dunker.setPosition(WIGGLE_POSITION1);
            dunker.setPosition(WIGGLE_POSITION2);
        }
    }
}




