package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.jvm.internal.CallableReference
import kotlin.reflect.KProperty
import kotlin.reflect.jvm.javaField

/**
 * Subclass wrapper of [Gamepad] with helper functions
 */
class GamepadExt(private val gamepad: Gamepad) : Gamepad() {
    val prev = Gamepad()

    fun update() = prev.copy(this).also { this.copy(gamepad) }
}

/* * * * * * * * * * * *\
 * GamepadExt helpers  *
\* * * * * * * * * * * */

inline fun Pair<GamepadExt, GamepadExt>.onEach(block: (GamepadExt) -> Unit) {
    block(first)
    block(second)
}

fun pressed(key: KProperty<*>): Boolean {
    val gamepad = (key as CallableReference).boundReceiver as GamepadExt
    return key.javaField!!.let { !it.getBoolean(gamepad.prev) && it.getBoolean(gamepad) }
}

fun anypressed(vararg keys: KProperty<*>): Boolean {
    for (key in keys) if (pressed(key)) return true
    return false
}

fun released(key: KProperty<*>): Boolean {
    val gamepad = (key as CallableReference).boundReceiver as GamepadExt
    return key.javaField!!.let { it.getBoolean(gamepad.prev) && !it.getBoolean(gamepad) }
}

fun anyreleased(vararg keys: KProperty<*>): Boolean {
    for (key in keys) if (released(key)) return true
    return false
}
