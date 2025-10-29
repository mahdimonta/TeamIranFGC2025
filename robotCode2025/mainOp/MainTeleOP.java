package org.firstinspires.ftc.teamcode.robotCode2025.mainOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotCode2025.imu.IMUClass;
import org.firstinspires.ftc.teamcode.robotCode2025.motorController.RobotController;
import org.firstinspires.ftc.teamcode.robotCode2025.vision.AprilTag;

@TeleOp
public class MainTeleOP extends LinearOpMode {

    IMUClass imu = new IMUClass();
    RobotController board = new RobotController();
    AprilTag Ap = new AprilTag();

    CRServo lArm,rArm;
    CRServo intackServo,accelerator;

    DcMotor shooter,winch;





    @Override
    public void runOpMode() throws InterruptedException {
        imu.imuInit(hardwareMap);
        imu.imu.resetYaw();

        board.chassisMotorInit(hardwareMap);
        Ap.initAprilTag(hardwareMap);

        intackServo = hardwareMap.get(CRServo.class,"intackServo");
        accelerator = hardwareMap.get(CRServo.class,"accelerator");

        lArm = hardwareMap.get(CRServo.class,"leftArm");
        rArm = hardwareMap.get(CRServo.class,"rightArm");
        rArm.setDirection(CRServo.Direction.REVERSE);

        shooter = hardwareMap.get(DcMotor.class,"shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winch = hardwareMap.get(DcMotor.class,"winch");
        winch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


        while (!isStopRequested() && opModeIsActive()){

            gamePad2();

            gamePad1();

            telemetry.update();

        }
        Ap.visionPortal.close();
    }





    boolean autoCam = false , handHeldCam=false , autoPilot = false;

    void gamePad2(){
        board.intakeAndElevator(gamepad2);

        if (gamepad2.y){
            autoCam=!autoCam;
            handHeldCam=false;
            sleep(60);
        }
        if (gamepad2.b){
            handHeldCam=!handHeldCam;
            autoCam=false;
            sleep(60);
        }
        if (gamepad2.x){
            autoPilot=!autoPilot;
            sleep(60);
        }
        if (autoPilot){
            telemetry.addData("autoPilot"," is on");
            Ap.driveToTag(board,telemetry);
        }

        if(handHeldCam){
            telemetry.addData("handHeldCam"," is on");
            Ap.trackingAprilTagHandHeld(gamepad2,telemetry);

        }

        if(autoCam){
            telemetry.addData("autoCam"," is on");
            Ap.trackingAprilTagAuto(telemetry,gamepad2);

        }

        double leftX = gamepad2.left_stick_x;
        double rightX = gamepad2.right_stick_x;
        lArm.setPower(-leftX);
        rArm.setPower(rightX);
    }

    boolean movementMode = false;

    void gamePad1(){

        if (gamepad1.dpad_up)
            intackServo.setPower(1);
        else if (gamepad1.dpad_down)
            intackServo.setPower(-1);
        else
            intackServo.setPower(0);

        if (gamepad1.left_trigger>0)
            accelerator.setPower(1);
        else if (gamepad1.right_trigger>0)
            accelerator.setPower(-1);
        else
            accelerator.setPower(0);
        shooterAndWinch();

        if(gamepad1.a){

            movementMode= !movementMode;
            sleep(60);
        }
        if(!movementMode){
            telemetry.addData("Movement Mode: ","Field Oriented Mode");
            board.fieldCentricDrive(gamepad1,telemetry,imu.imuGetYaw());
        }else {
            telemetry.addData("Movement Mode: ","Normal Mode");
            board.normalHolonomicDrive(gamepad1,telemetry);
        }
    }
    public void shooterAndWinch(){

        if (gamepad1.left_bumper)
            shooter.setPower(-1);
        else  shooter.setPower(0);

        if (gamepad1.right_bumper)
            winch.setPower(1);
        else winch.setPower(0);




    }

}
