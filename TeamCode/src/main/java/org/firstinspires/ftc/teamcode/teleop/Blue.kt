package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.EventLoop
import org.firstinspires.ftc.teamcode.robot.GamepadExt
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.BACKMID
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.GROUND
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.LOW
import org.firstinspires.ftc.teamcode.robot.subsystems.Arm.States.MID
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw
import org.firstinspires.ftc.teamcode.robot.subsystems.Claw.States.CLOSED
import org.firstinspires.ftc.teamcode.robot.subsystems.DriveExt
import org.firstinspires.ftc.teamcode.robot.sync
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline
import org.firstinspires.ftc.teamcode.vision.ConeDetectionPipeline.BLUE
import org.firstinspires.ftc.teamcode.vision.createWebcam
import org.openftc.easyopencv.OpenCvWebcam

@TeleOp(group = "a")
class Blue : LinearOpMode() {
    private lateinit var claw: Claw
    private lateinit var arm: Arm
    private lateinit var drive: DriveExt
    private lateinit var webcam: OpenCvWebcam
    private val pipeline = ConeDetectionPipeline(BLUE)
    private val pickPID = PIDController(0.001, 0.0, 0.0).apply { setPoint = 0.0 }

    private val tm = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    override fun runOpMode() {
        arm = Arm(hardwareMap)
        claw = Claw(hardwareMap)
        drive = DriveExt(hardwareMap)
        webcam = createWebcam(hardwareMap, pipeline = pipeline)

        val gp1 = GamepadExt(gamepad1)
        val gp2 = GamepadExt(gamepad2)
        val gamepads = gp1 to gp2

        EventLoop(::opModeIsActive, tm).apply {
            updates += listOf(arm::update,
                { drive.update(gamepads) },
                gamepads::sync,
                { tm.update(); Unit })

            onAnyPressed(gp1::left_bumper, gp2::left_bumper) { claw.change() }
            onAnyPressed(gp1::dpad_up, gp2::dpad_up) { arm.state = Arm.next(arm.state) }
            onAnyPressed(gp1::dpad_down, gp2::dpad_down) { arm.state = Arm.prev(arm.state) }
            onAnyPressed(gp1::a, gp2::a) { arm.state = GROUND }
            onAnyPressed(gp1::x, gp2::x) { arm.state = LOW }
            onAnyPressed(gp1::y, gp2::y) { arm.state = MID }
            onAnyPressed(gp1::b, gp2::b) { arm.state = BACKMID }

            /* aim at cone */
            onAnyPressed(gp1::right_bumper, gp2::right_bumper) {
                claw.state = Claw.OPENED
                singleEvents += { claw.state != CLOSED } to {
                    if (pipeline.detected) {
                        pickPID.setPID(TeleOp3.pP, TeleOp3.pI, TeleOp3.pD)
                        gp1.right_stick_x = pickPID.calculate(pipeline.error).toFloat()
                    }
                }
            }
        }.also { waitForStart() }.run()
    }
}
