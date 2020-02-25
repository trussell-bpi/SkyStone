package org.firstinspires.ftc.teamcode;
import java.lang.Math;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name="badAuto",group="Move")

public abstract class badAuto extends LinearOpMode {
    private DcMotor FR,FL,BR,BL;
    private ElapsedTime     runtime = new ElapsedTime();
    private void moveWithEncoder (float distance) {
        //reset encoders because I think the ticks that they are on are supposed to be reset
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //use encoder
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        double circumference= Math.PI*3.85827;
        int targetTicks=(int)((distance/circumference)*383.6);
        int currentPos=(FR.getCurrentPosition()+FL.getCurrentPosition()+BR.getCurrentPosition()+BL.getCurrentPosition())/4;
        int ticksToGo=targetTicks-currentPos;
        runtime.reset();
        FR.setPower(1);
        FL.setPower(1);
        BR.setPower(1);
        BL.setPower(1);
        while (ticksToGo>10) {
            currentPos=(FR.getCurrentPosition()+FL.getCurrentPosition()+BR.getCurrentPosition()+BL.getCurrentPosition())/4;
            ticksToGo=targetTicks-currentPos;
            if (runtime.seconds()>5) {
                break;
            }
        }
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    @Override
    public void runOpMode() {
        FR=hardwareMap.dcMotor.get("Front Right Motor");
        FL=hardwareMap.dcMotor.get("Front Left Motor");
        BR=hardwareMap.dcMotor.get("Back Right Motor");
        BL=hardwareMap.dcMotor.get("Back Left Motor");
        moveWithEncoder(5);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

    }

}
