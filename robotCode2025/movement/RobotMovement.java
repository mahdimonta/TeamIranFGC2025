package org.firstinspires.ftc.teamcode.robotCode2025.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotCode2025.imu.IMUClass;


public class RobotMovement {
    DcMotor FRMotor,FLMotor,BRMotor,BLMotor;
    double axisY,axisX, rot,frm, flm, brm, blm ;

    public void chassisMotorInit(HardwareMap hardwareMap){

        FRMotor = hardwareMap.get(DcMotor.class,"FR");
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor = hardwareMap.get(DcMotor.class,"FL");
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BRMotor = hardwareMap.get(DcMotor.class,"BR");
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BLMotor = hardwareMap.get(DcMotor.class,"BL");
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BLMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void normalHolonomicDrive(Gamepad gamepad1, Telemetry telemetry){




        axisY = gamepad1.left_stick_y;
        axisX = -gamepad1.left_stick_x;
        rot = -gamepad1.right_stick_x;

        flm = axisY + axisX + rot;
        frm = axisY - axisX - rot;
        blm = axisY - axisX + rot;
        brm = axisY + axisX - rot;



        double maxSpeed = Math.max(Math.abs(flm), Math.abs(frm));
        maxSpeed = Math.max(maxSpeed, Math.abs(blm));
        maxSpeed = Math.max(maxSpeed, Math.abs(brm));

        if (maxSpeed > 1.0) {
            flm /= maxSpeed;
            frm /= maxSpeed;
            blm /= maxSpeed;
            brm /= maxSpeed;
        }

        //simple holonomic drive
        /*

           flm = -axisY+axisX;
           frm = axisY+axisX;
           blm = -axisY-axisX;
           brm = axisY-axisX;

           if(rot >0){

                flm = (flm + rot);
                frm = (-(frm - rot));
                blm = (blm + rot);
                brm = (-(brm - rot));

            }
            if(rot <0){

                flm = (-(flm - rot));
                frm = (frm + rot);
                blm = (-(blm - rot));
                brm = (brm +  rot);

            }
        */

        FLMotor.setPower(flm);
        FRMotor.setPower(frm);
        BLMotor.setPower(blm);
        BRMotor.setPower(brm);

        telemetry.addData("flm : ",flm);
        telemetry.addData("frm : ",frm);
        telemetry.addData("blm : ",blm);
        telemetry.addData("brm : ",brm);



    }

    public void fieldCentricDrive(Gamepad gamepad1, Telemetry telemetry,double gyroAngel){

        axisY = gamepad1.left_stick_y;
        axisX = -gamepad1.left_stick_x;
        rot = -gamepad1.right_stick_x;

        double joystickMagnitude = Math.sqrt(axisX * axisX + axisY * axisY);
        double joystickAngle = Math.atan2(axisX, axisY);

        if (joystickMagnitude < 0.05) {
            joystickMagnitude = 0.0;
        }

        if (joystickMagnitude > 1.0) {
            joystickMagnitude = 1.0;
        }

        double adjustedAngle = joystickAngle + gyroAngel;

        double Vx_robot = joystickMagnitude * Math.sin(adjustedAngle);
        double Vy_robot = joystickMagnitude * Math.cos(adjustedAngle);

        flm = Vy_robot +Vx_robot + rot;
        frm = Vy_robot -Vx_robot - rot;
        blm = Vy_robot -Vx_robot + rot;
        brm = Vy_robot +Vx_robot - rot;


        double maxSpeed = Math.max(Math.abs(flm), Math.abs(frm));
        maxSpeed = Math.max(maxSpeed, Math.abs(blm));
        maxSpeed = Math.max(maxSpeed, Math.abs(brm));

        if (maxSpeed > 1.0) {
            flm /= maxSpeed;
            frm /= maxSpeed;
            blm /= maxSpeed;
            brm /= maxSpeed;
        }



        FLMotor.setPower(flm);
        FRMotor.setPower(frm);
        BLMotor.setPower(blm);
        BRMotor.setPower(brm);

        telemetry.addData("gyro yaw : ",gyroAngel);
        telemetry.addData("joy angel : ", joystickAngle);
        telemetry.addData("flm : ",flm);
        telemetry.addData("frm : ",frm);
        telemetry.addData("blm : ",blm);
        telemetry.addData("brm : ",brm);
    }

    //get right joy stick angel (degree)
    public double joyStickAngel() {
        if (Math.abs(axisX) < 0.1 && Math.abs(axisY) < 0.1)
            return 0;
        else
            return Math.atan2(axisX, -axisY);
    }





}
