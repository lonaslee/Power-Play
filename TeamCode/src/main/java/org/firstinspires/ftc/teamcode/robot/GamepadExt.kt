package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.reflect.KProperty0
import kotlin.reflect.KProperty1
import kotlin.reflect.jvm.javaField

/**
 * Subclass wrapper of [Gamepad] that has some helper functions.
 */
class GamepadExt(private val gamepad: Gamepad) : Gamepad() {
    private val prev = Gamepad()

    infix fun pressed(key: KProperty0<Boolean>) = key.javaField!!.let {
        !(it.get(prev) as Boolean) && it.get(this) as Boolean
    }

    infix fun pressed(key: KProperty1<Gamepad, Boolean>) = !key.get(prev) && key.get(this)

    infix fun released(key: KProperty0<Boolean>) = key.javaField!!.let {
        it.get(prev) as Boolean && (it.get(this) as Boolean)
    }

    infix fun released(key: KProperty1<Gamepad, Boolean>) = key.get(prev) && !key.get(this)

    fun update() = prev.copy(this).also { this.copy(gamepad) }
}

inline fun Pair<GamepadExt, GamepadExt>.onEach(block: (GamepadExt) -> Unit) {
    block(first)
    block(second)
}
