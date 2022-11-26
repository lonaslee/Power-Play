package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Arm3.Height.*
import kotlin.math.cos

@Config
class Arm3(hardwareMap: HardwareMap, private val telemetry: Telemetry) {
    private val topmotor = hardwareMap[RobotConfig.TOP_LIFT.s] as DcMotorEx
    private val lowmotor = hardwareMap[RobotConfig.LOW_LIFT.s] as DcMotorEx
    private val motors = listOf(topmotor, lowmotor).onEach {
        it.direction = DcMotorSimple.Direction.REVERSE
        it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        it.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    private val lowControl = PIDController(kP, kI, kD)
    private val midControl = PIDController(mP, mI, mD)
    private val backControl = PIDFController(bP, bI, bD, bF)
    private val downControl = PIDFController(dP, dI, dD, dF)

    val curOutput get() = lowmotor.currentPosition

    private val feedforward
        get() = when (height) {
            LOW              -> kCos
            MID              -> mCos
            BACKMID, BACKLOW -> bCos
            else             -> 0.0
        } * cos(Math.toRadians((height.pos.toDouble() - 180) / TICKS_IN_DEGREES))

    private var desiredHeight = GROUND
    var height = GROUND
        set(value) {
            if (value == height) return
            goingDown = height > value
            field = value
        }

    private var goingDown = false

    fun update() {
        lowControl.setPID(kP, kI, kD)
        midControl.setPID(mP, mI, mD)
        backControl.setPIDF(bP, bI, bD, bF)
        downControl.setPIDF(dP, dI, dD, dF)

        val pid = when (height) {
            LOW              -> lowControl
            MID              -> midControl
            BACKMID, BACKLOW -> backControl
            else             -> downControl
        }.calculate(curOutput.toDouble(), height.pos.toDouble())

        motors.forEach { it.power = pid + feedforward }

        telemetry.addData("_pos", curOutput)
        telemetry.addData("_ref", height.pos)
        telemetry.addData("pid", pid)
        telemetry.addData("ff", feedforward)
    }

    fun adjustAccordingTo(gp1: GamepadExt, gp2: GamepadExt) {
        if (gp1 pressed gp1::y || gp2 pressed gp2::y) height = height.next
        else if (gp1 pressed gp1::a || gp2 pressed gp2::a) height = height.prev
        else if (gp1 pressed gp1::x || gp2 pressed gp2::x) height = MID
        else if (gp1 pressed gp1::b || gp2 pressed gp2::b) height = LOW
        update()
    }

    infix fun adjustAccordingTo(gps: Pair<GamepadExt, GamepadExt>) =
        adjustAccordingTo(gps.component1(), gps.component2())

    companion object {
        const val kP = 0.002 // done
        const val kI = 0.0
        const val kD = 0.0
        const val kCos = 0.005

        @JvmField var mP = 0.002
        @JvmField var mI = 0.0
        @JvmField var mD = 0.0005
        @JvmField var mCos = 0.0

        var bP = 0.002
        var bI = 0.0
        var bD = 0.000015
        var bCos = 0.13
        var bF = 0.0

        var dP = 0.001
        var dI = 0.0
        var dD = 0.00001
        var dF = 0.0

        const val TICKS_IN_DEGREES = 260 / 90.0
    }

    enum class Height(val pos: Int) {
        GROUND(5), LOW(240), MID(340), BACKMID(480), BACKLOW(560), STACK(60);

        val next get() = values()[if (ordinal > 3) 4 else ordinal + 1]
        val prev get() = values()[if (ordinal == 0) 0 else ordinal - 1]
    }
}
