package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.DriveFunctions;

public class linearActuator implements Hang{

    private final float HANG_POWER = (float) 1.0;
    private final int HANG_DISTANCE = 3100;
    private final int MAX_DISTANCE = 3100;

    private DcMotor hanger;

    public linearActuator(DcMotor linearActuator)
    {
        hanger = linearActuator;
    }

    public void up() throws InterruptedException
    {
        DriveFunctions.oneMotorEncoder(hanger, HANG_POWER, HANG_DISTANCE);
        hanger.setPower(HANG_POWER);
        Thread.sleep(1500);
        hanger.setPower(0.0);
    }

    public void down() throws InterruptedException
    {
        DriveFunctions.oneMotorEncoder(hanger, -HANG_POWER/2, -HANG_DISTANCE);
        hanger.setPower(-HANG_POWER);
        Thread.sleep(1500);
        hanger.setPower(0.0);
    }
}
