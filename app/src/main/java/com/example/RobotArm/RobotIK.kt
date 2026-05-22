package com.example.RobotArm

import android.util.Log
import kotlin.math.*

// RobotIK.kt
// Complete rewrite, fixes flange sign bug and improves debugging info.
// - Expects model.dhParams (list/array of DH param objects for 6 joints)
// - flange.thetaOffset is applied as Rz(thetaOffset) and removed via transpose(Rflange)
// - returns List<FloatArray> where each FloatArray has length 6 (degrees for joints 1..6)

class RobotIK(
    private val model: RobotModel,
    private val fk: RobotFK,
    minAnglesDeg: FloatArray? = null,
    maxAnglesDeg: FloatArray? = null,
    private val verbose: Boolean = false) {
    private val Debug_Tag = "RobotIK"
    init {
        if (minAnglesDeg != null) require(minAnglesDeg.size == 7)
        if (maxAnglesDeg != null) require(maxAnglesDeg.size == 7)
    }

    private val minAnglesDeg = minAnglesDeg
    private val maxAnglesDeg = maxAnglesDeg

    private val dh = model.dhParams.take(6)
    private val flange = model.flange
    private val baseOffset = model.baseXOffset

    private val minAnglesRad: FloatArray? = minAnglesDeg?.let { arr ->
        FloatArray(7) { i -> if (i == 0) 0f else degToRad(arr[i]) }
    }
    private val maxAnglesRad: FloatArray? = maxAnglesDeg?.let { arr ->
        FloatArray(7) { i -> if (i == 0) 0f else degToRad(arr[i]) }
    }

    private fun validAnglesDeg(j: FloatArray): Boolean {
        val minA = minAnglesDeg
        val maxA = maxAnglesDeg

        if (minA == null || maxA == null) return true
        for (i in 1 until 7) {
            if (j[i] <= minA[i] || j[i] >= maxA[i]) return false
        }
        return true
    }


    // --- math helpers ---
    private fun degToRad(d: Float) = (d * Math.PI / 180.0).toFloat()
    private fun radToDeg(r: Float) = Math.toDegrees(r.toDouble()).toFloat()

    private fun clampAngleRad(a: Float): Float {
        var x = a
        val two = (2 * Math.PI).toFloat()
        if (x > Math.PI) x -= two
        else if (x <= -Math.PI) x += two
        return x
    }
    fun internalToUI(degInternal: FloatArray): FloatArray {
        val out = FloatArray(7)
        out[0] = 0f
        for (i in 1..6) out[i] = degInternal[i] * model.jointDirection[i]
        return out
    }

    fun roundIfTiny(v: Float) = if (abs(v) < 1e-3f) 0f else (round(v * 100000) / 100000.0f)
    fun normDeg(a: Float): Float {
        var x = a
        if (x <= -180f) x += 360f
        else if (x > 180f) x -= 360f
        return x
    }
    fun closeEnough(a: FloatArray, b: FloatArray, tolDeg: Float = 1e-4f): Boolean {
        for (i in 1..6) if (abs(a[i] - b[i]) > tolDeg) return false
        return true
    }

    fun printMat3(label: String, R: Array<FloatArray>) {
        println("$label =")
        println("  [ %.6f  %.6f  %.6f ]".format(R[0][0], R[0][1], R[0][2]))
        println("  [ %.6f  %.6f  %.6f ]".format(R[1][0], R[1][1], R[1][2]))
        println("  [ %.6f  %.6f  %.6f ]".format(R[2][0], R[2][1], R[2][2]))
    }
    fun printMat4(label: String, T: Array<FloatArray>) {
        println("$label =")
        println("  [ %.3f  %.3f  %.3f  %.3f ]".format(T[0][0], T[0][1], T[0][2], T[0][3]))
        println("  [ %.3f  %.3f  %.3f  %.3f ]".format(T[1][0], T[1][1], T[1][2], T[1][3]))
        println("  [ %.3f  %.3f  %.3f  %.3f ]".format(T[2][0], T[2][1], T[2][2], T[2][3]))
        println("  [ %.3f  %.3f  %.3f  %.3f ]".format(T[3][0], T[3][1], T[3][2], T[3][3]))
    }


    // --- 4x4 DH transform (standard DH) (row-major) ---
    private fun dhMatrix(a: Float, alpha: Float, d: Float, theta: Float): Array<FloatArray> {
        val ct = cos(theta)
        val st = sin(theta)
        val ca = cos(alpha)
        val sa = sin(alpha)

        return arrayOf(
            floatArrayOf(ct, -st * ca, st * sa, a * ct),
            floatArrayOf(st, ct * ca, -ct * sa, a * st),
            floatArrayOf(0f, sa, ca, d),
            floatArrayOf(0f, 0f, 0f, 1f)
        )
    }

    fun inverse3(m: Array<FloatArray>): Array<FloatArray> {
        val a = m[0][0];
        val b = m[0][1];
        val c = m[0][2]
        val d = m[1][0];
        val e = m[1][1];
        val f = m[1][2]
        val g = m[2][0];
        val h = m[2][1];
        val i = m[2][2]

        val det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

        if (abs(det) < 1e-8f)
            throw IllegalArgumentException("Matrix is singular, cannot invert")

        val invDet = 1f / det

        return arrayOf(
            floatArrayOf(
                (e * i - f * h) * invDet,
                (c * h - b * i) * invDet,
                (b * f - c * e) * invDet
            ),
            floatArrayOf(
                (f * g - d * i) * invDet,
                (a * i - c * g) * invDet,
                (c * d - a * f) * invDet
            ),
            floatArrayOf(
                (d * h - e * g) * invDet,
                (b * g - a * h) * invDet,
                (a * e - b * d) * invDet
            )
        )
    }

    fun inverse4(m: Array<FloatArray>): Array<FloatArray> {
        val inv = FloatArray(16)
        val a = FloatArray(16)

        // convert to 1D for easier indexing
        for (r in 0..3) {
            for (c in 0..3) {
                a[r * 4 + c] = m[r][c]
            }
        }

        inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] -
                a[9] * a[6] * a[15] + a[9] * a[7] * a[14] +
                a[13] * a[6] * a[11] - a[13] * a[7] * a[10]

        inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] +
                a[8] * a[6] * a[15] - a[8] * a[7] * a[14] -
                a[12] * a[6] * a[11] + a[12] * a[7] * a[10]

        inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] -
                a[8] * a[5] * a[15] + a[8] * a[7] * a[13] +
                a[12] * a[5] * a[11] - a[12] * a[7] * a[9]

        inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] +
                a[8] * a[5] * a[14] - a[8] * a[6] * a[13] -
                a[12] * a[5] * a[10] + a[12] * a[6] * a[9]

        inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] +
                a[9] * a[2] * a[15] - a[9] * a[3] * a[14] -
                a[13] * a[2] * a[11] + a[13] * a[3] * a[10]

        inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] -
                a[8] * a[2] * a[15] + a[8] * a[3] * a[14] +
                a[12] * a[2] * a[11] - a[12] * a[3] * a[10]

        inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] +
                a[8] * a[1] * a[15] - a[8] * a[3] * a[13] -
                a[12] * a[1] * a[11] + a[12] * a[3] * a[9]

        inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] -
                a[8] * a[1] * a[14] + a[8] * a[2] * a[13] +
                a[12] * a[1] * a[10] - a[12] * a[2] * a[9]

        inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] -
                a[5] * a[2] * a[15] + a[5] * a[3] * a[14] +
                a[13] * a[2] * a[7] - a[13] * a[3] * a[6]

        inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] +
                a[4] * a[2] * a[15] - a[4] * a[3] * a[14] -
                a[12] * a[2] * a[7] + a[12] * a[3] * a[6]

        inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] -
                a[4] * a[1] * a[15] + a[4] * a[3] * a[13] +
                a[12] * a[1] * a[7] - a[12] * a[3] * a[5]

        inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] +
                a[4] * a[1] * a[14] - a[4] * a[2] * a[13] -
                a[12] * a[1] * a[6] + a[12] * a[2] * a[5]

        inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] +
                a[5] * a[2] * a[11] - a[5] * a[3] * a[10] -
                a[9] * a[2] * a[7] + a[9] * a[3] * a[6]

        inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] -
                a[4] * a[2] * a[11] + a[4] * a[3] * a[10] +
                a[8] * a[2] * a[7] - a[8] * a[3] * a[6]

        inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] +
                a[4] * a[1] * a[11] - a[4] * a[3] * a[9] -
                a[8] * a[1] * a[7] + a[8] * a[3] * a[5]

        inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] -
                a[4] * a[1] * a[10] + a[4] * a[2] * a[9] +
                a[8] * a[1] * a[6] - a[8] * a[2] * a[5]

        var det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12]

        if (abs(det) < 1e-8f)
            throw IllegalArgumentException("Matrix is singular, cannot invert")

        det = 1.0f / det

        // convert back to 4x4 array
        val out = Array(4) { FloatArray(4) }
        for (i in 0 until 16) {
            out[i / 4][i % 4] = inv[i] * det
        }
        return out
    }

    private fun mul4(a: Array<FloatArray>, b: Array<FloatArray>): Array<FloatArray> {
        val r = Array(4) { FloatArray(4) }
        for (i in 0..3) {
            for (j in 0..3) {
                var s = 0f
                for (k in 0..3) {
                    s += a[i][k] * b[k][j]
                }
                r[i][j] = s
            }
        }
        return r
    }

    private fun mul3(A: Array<FloatArray>, B: Array<FloatArray>): Array<FloatArray> {
        val R = Array(3) { FloatArray(3) }
        for (i in 0..2) {
            for (j in 0..2) {
                var s = 0f
                for (k in 0..2) {
                    s += A[i][k] * B[k][j]
                }
                R[i][j] = s
            }
        }
        return R
    }

    private fun R_matrix(T: Array<FloatArray>): Array<FloatArray> {
        return arrayOf(
            floatArrayOf(T[0][0], T[0][1], T[0][2]),
            floatArrayOf(T[1][0], T[1][1], T[1][2]),
            floatArrayOf(T[2][0], T[2][1], T[2][2])
        )
    }

    private fun transpose3(m: Array<FloatArray>): Array<FloatArray> {
        return arrayOf(
            floatArrayOf(m[0][0], m[1][0], m[2][0]),
            floatArrayOf(m[0][1], m[1][1], m[2][1]),
            floatArrayOf(m[0][2], m[1][2], m[2][2])
        )
    }

    private fun transpose4(m: Array<FloatArray>): Array<FloatArray> {
        return arrayOf(
            floatArrayOf(m[0][0], m[1][0], m[2][0], m[3][0]),
            floatArrayOf(m[0][1], m[1][1], m[2][1], m[3][1]),
            floatArrayOf(m[0][2], m[1][2], m[2][2], m[3][2]),
            floatArrayOf(m[0][3], m[1][3], m[2][3], m[3][3])
        )
    }


    // ----------------------------- Main IK --------------------------------
    fun solveIK(T: Array<FloatArray>): List<FloatArray> {
        val solutions = mutableListOf<FloatArray>()
        val Ex = T[0][3] - baseOffset
        val Ey = T[1][3]
        val Ez = T[2][3]

        if (verbose) {
            printMat4("T06 with Flanged", T)
            Log.i("IK","EE_Cor = [$Ex,$Ey,$Ez]")
//            val orient = fk.getTCPOrientation(T)
//            val euler = fk.getTCPEulerZYX(orient)
//            val EZ_Cur = roundIfTiny(euler[0])
//            val EY_Cur = roundIfTiny(euler[1])
//            val EX_Cur = roundIfTiny(euler[2])
//            Log.i(Debug_Tag,"EZ_Cur = $EZ_Cur, EY_Cur = $EY_Cur, EX_Cur = $EX_Cur")
        }

        // deflange
        val Tflange = dhMatrix(flange.a, flange.alpha, flange.d, flange.thetaOffset)
        val T06 = mul4(T, inverse4(Tflange))
        if (verbose) printMat4("T06_deFlanged", T06)

        // wrist center
        val d6 = dh[5].d + flange.d
        val zx = T06[0][2]; val zy = T06[1][2]; val zz = T06[2][2]
        val wx = Ex - d6 * zx
        val wy = Ey - d6 * zy
        val wz = Ez - d6 * zz
        if (verbose) Log.i("IK","Wrist Center = [$wx,$wy,$wz]")

        // t1 candidates
        val t1a = atan2(wy, wx)
        val t1b = clampAngleRad(t1a + Math.PI.toFloat())
        val theta1Candidates = listOf(t1a, t1b)

        // robot geometry shortcuts
        val a2 = dh[1].a
        val a3 = dh[2].a
        val d4 = dh[3].d
        val l5 = sqrt(a3*a3 + d4*d4) // a3_real

        for (t1 in theta1Candidates) {
            if (verbose) Log.i("IK_J1","For t1 = ${radToDeg(t1)}")

            // T01 and R01^T
            val T01 = dhMatrix(dh[0].a, dh[0].alpha, dh[0].d, t1 + dh[0].thetaOffset)
            val R01T = transpose3(R_matrix(T01))

            // wrist relative to joint1 origin then rotate into joint1 frame -> then we want coords relative to joint2 origin
            val dx = wx - T01[0][3]
            val dy = wy - T01[1][3]
            val dz = wz - T01[2][3]

            val x1 = R01T[0][0]*dx + R01T[0][1]*dy + R01T[0][2]*dz
            val y1 = R01T[1][0]*dx + R01T[1][1]*dy + R01T[1][2]*dz
            val z1 = R01T[2][0]*dx + R01T[2][1]*dy + R01T[2][2]*dz
            if (verbose) Log.i("IK_J2","Wrist relative to J2 (x1,y1,z1) = [$x1,$y1,$z1]")

            // choose planar triangle axes (your convention uses r = x1, s = y1)
            val r = x1
            val s = y1
            val h = sqrt(r*r + s*s)

            // guard cos range (numerical)
            fun clamp01(x: Float): Float {
                return when {
                    x > 1f -> 1f
                    x < -1f -> -1f
                    else -> x
                }
            }

            val cosWBC = (r*r + s*s + a2*a2 - a3*a3 - d4*d4) / (2f * h * a2)
            val cosWBCc = clamp01(cosWBC)
            val sinWBC = sqrt(max(0f, 1f - cosWBCc*cosWBCc))

            val t2a = atan2(s, r) + atan2(sinWBC, cosWBCc) - dh[1].thetaOffset
            val t2b = atan2(s, r) + atan2(-sinWBC, cosWBCc) - dh[1].thetaOffset

            if (verbose) Log.i("IK_J2","t2 candidates (deg) = ${radToDeg(t2a)}, ${radToDeg(t2b)}")

            // gamma for theta3 (use l5 as diagonal)
            val cosGamma = -(a2*a2 + l5*l5 - h*h) / (2f * a2 * l5)
            val cosGclamped = clamp01(cosGamma)
            val sinGamma = sqrt(max(0f, 1f - cosGclamped*cosGclamped))

            val t3a = atan2(d4, a3) + atan2(sinGamma, cosGclamped) - dh[2].thetaOffset
            val t3b = atan2(d4, a3) + atan2(-sinGamma, cosGclamped) - dh[2].thetaOffset

            if (verbose) Log.i("IK_J3","t3 candidates (deg) = ${radToDeg(t3a)}, ${radToDeg(t3b)}")

            val branch23 = listOf(
                Pair(t2a, t3a), Pair(t2a, t3b),
                Pair(t2b, t3a), Pair(t2b, t3b)
            )

            // precompute R06
            val R06 = R_matrix(T06)

            for ((t2, t3) in branch23) {
                if (verbose) Log.i("IK_branch","trying t2=${radToDeg(t2)} t3=${radToDeg(t3)}")

                // build T02 and T03
                val T02 = mul4(T01, dhMatrix(a2, dh[1].alpha, dh[1].d, t2 + dh[1].thetaOffset))
                val T03 = mul4(T02, dhMatrix(a3, dh[2].alpha, dh[2].d, t3 - dh[2].thetaOffset))

                val R03 = R_matrix(T03)
                val R36 = mul3(transpose3(R03), R06)

                if (verbose) {
                    printMat4("T03", T03)
                    printMat3("R03", R03)
                    printMat3("R36", R36)
                }

                val r11 = R36[0][0]; val r12 = R36[0][1]; val r13 = R36[0][2]
                val r21 = R36[1][0]; val r22 = R36[1][1]; val r23 = R36[1][2]
                val r31 = R36[2][0]; val r32 = R36[2][1]; val r33 = R36[2][2]

                val insideAtan = sqrt(1f - r33*r33)
                val t5a = atan2(insideAtan, r33)
                val t5b = atan2(-insideAtan, r33)
                val theta5Candidates = listOf(t5a, t5b)
                if (verbose) {
                    Log.i("IK_branch","r33=${r33}")
                    Log.i("IK_J5","t5a=${radToDeg(t5a)}, t5b=${radToDeg(t5b)}")
                }

                for (t5 in theta5Candidates) {
                    if (verbose) Log.i("IK_J4","For t5 = ${radToDeg(t5)}")

                    // giữ nguyên cách tính t4s, t6s
                    val t4s: Float
                    val t6s: Float

                    if (abs(r33) == 1.0f) {
                        t4s = atan2(r21, r11)
                        t6s = 0f

                        if (verbose) Log.i("IK_J4", "t4s=${radToDeg(t4s)}")
                    }
                    else {
                        t4s = atan2(r23, r13)
                        t6s = atan2(r32, -r31)

                        if (verbose) {
                            Log.i("IK_J4","t4s=${radToDeg(t4s)}")
                            Log.i("IK_J4","t6s=${radToDeg(t6s)}")
                        }
                    }

                    // thêm branch mới
                    val t4a = t4s
                    val t4b = t4s - Math.PI.toFloat()

                    val t6a = t6s
                    val t6b = t6s - Math.PI.toFloat()

                    // 4 combination
                    val wristBranches = listOf(
                        Pair(t4a, t6a),
                        Pair(t4a, t6b),
                        Pair(t4b, t6a),
                        Pair(t4b, t6b)
                    )

                    for ((t4, t6) in wristBranches) {

                        val sol = floatArrayOf(
                            0f,
                            radToDeg(t1) * model.jointDirection[1],
                            radToDeg(t2) * model.jointDirection[2],
                            radToDeg(t3) * model.jointDirection[3],
                            radToDeg(t4) * model.jointDirection[4],
                            radToDeg(t5) * model.jointDirection[5],
                            radToDeg(t6) * model.jointDirection[6]
                        )

                        for (i in 1..6) sol[i] = normDeg(sol[i])

                        // check joint limit
                        if (!validAnglesDeg(sol)) continue

                        // add to full solution list
                        solutions.add(sol)

                        if (verbose) {
                            Log.i("IK_J4", "Added wrist branch: t4=${radToDeg(t4)}, t6=${radToDeg(t6)}")
                        }
                    }
                }
            }
        }

        // deduplicate: remove near-identical solutions (angle-wise)
        val final = mutableListOf<FloatArray>()

        for (s in solutions) {
            var dup = false
            for (f in final){
                if (closeEnough(s, f, 1e-3f)) {
                    dup = true
                    break
                }
            }
            if (!dup) final.add(s)
        }

        if (verbose) {
            Log.i("IK", "TOTAL SOLUTIONS = ${final.size}")
            for ((i, sol) in final.withIndex()) {
                Log.i("IK","SOL #$i = ${sol.joinToString(", ") { String.format("%.6f°", it) }}")
            }
        }

        return final
    }

    fun verifySolution(jointDeg: FloatArray, T_target: Array<FloatArray>): Boolean {
        val rotTol = 0.05f
        val T_calc = fk.forwardKinematicsDeg(jointDeg)
        val Ori_calc = fk.getTCPOrientation(T_calc)
        val Ori_tar = fk.getTCPOrientation(T_target)

        val euler_calc = fk.getTCPEulerZYX(Ori_calc)
        val euler_tar = fk.getTCPEulerZYX(Ori_tar)
        val ok = compareTCP(T_calc, T_target)

        val dz = roundIfTiny(euler_calc[0] - euler_tar[0])
        val dy = roundIfTiny(euler_calc[1] - euler_tar[1])
        val dx = roundIfTiny(euler_calc[2] - euler_tar[2])
        val EulerErr_ZYX = floatArrayOf(dz,dy,dx)

        var validCount = 0
        for (i in 0..2){
            if (abs(EulerErr_ZYX[i])  <= rotTol){
                validCount +=1
            }
        }
        if (verbose) Log.i("IK_VERIFY", "EulerErr_ZYX = [${ EulerErr_ZYX[0]}, ${ EulerErr_ZYX[1]}, ${ EulerErr_ZYX[2]}]")
        if ((validCount==3) && ok) {
            if (verbose){
                println("=== T06_calculated ===")
                Log.i("IK_VERIFY", "Joints = ${jointDeg.joinToString()}")
                Log.i("IK_VERIFY", "Match = $ok")
                for (i in 0..3) {
                    println(T_calc[i].joinToString("\t") { "%.3f".format(it) })
                }
                Log.i("IK_VERIFY", "POSITION TARGET = [${T_target[0][3]}, ${T_target[1][3]}, ${T_target[2][3]}]")
                Log.i("IK_VERIFY", "POSITION CALC   = [${T_calc[0][3]}, ${T_calc[1][3]}, ${T_calc[2][3]}]")
                println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler_calc[0], euler_calc[1], euler_calc[2]))
            }
            return true
        }
        else return false
    }

    fun verify_Orient_Solution(jointDeg: FloatArray, T_target: Array<FloatArray>): Boolean {
        val rotTol = 0.5f
        val T_calc = fk.forwardKinematicsDeg(jointDeg)
        val Ori_calc = fk.getTCPOrientation(T_calc)
        val Ori_tar = fk.getTCPOrientation(T_target)

        val euler_calc = fk.getTCPEulerZYX(Ori_calc)
        val euler_tar = fk.getTCPEulerZYX(Ori_tar)
        val ok = compareTCP(T_calc, T_target)

        val dz = roundIfTiny(euler_calc[0] - euler_tar[0])
        val dy = roundIfTiny(euler_calc[1] - euler_tar[1])
        val dx = roundIfTiny(euler_calc[2] - euler_tar[2])
        val EulerErr_ZYX = floatArrayOf(dz,dy,dx)

        var validCount = 0
        for (i in 0..2){
            if (abs(EulerErr_ZYX[i])  <= rotTol){
                validCount +=1
            }
        }
        if (verbose) Log.i("IK_VERIFY", "EulerErr_ZYX = [${ EulerErr_ZYX[0]}, ${ EulerErr_ZYX[1]}, ${ EulerErr_ZYX[2]}]")
        if ((validCount==3) && ok) {
            if (verbose){
                println("=== T06_calculated ===")
                Log.i("IK_VERIFY", "Joints = ${jointDeg.joinToString()}")
                Log.i("IK_VERIFY", "Match = $ok")
                for (i in 0..3) {
                    println(T_calc[i].joinToString("\t") { "%.3f".format(it) })
                }
                Log.i("IK_VERIFY", "POSITION TARGET = [${T_target[0][3]}, ${T_target[1][3]}, ${T_target[2][3]}]")
                Log.i("IK_VERIFY", "POSITION CALC   = [${T_calc[0][3]}, ${T_calc[1][3]}, ${T_calc[2][3]}]")
                println("Euler ZYX = (Ez=%.3f, Ey=%.3f, Ex=%.3f)".format(euler_calc[0], euler_calc[1], euler_calc[2]))
            }
            return true
        }
        else return false
    }


    fun compareTCP(
        T1: Array<FloatArray>,
        T2: Array<FloatArray>,
        posTol: Float = 0.5f,   // mm

    ): Boolean {
        val dx = roundIfTiny(T1[0][3] - T2[0][3])
        val dy = roundIfTiny(T1[1][3] - T2[1][3])
        val dz = roundIfTiny(T1[2][3] - T2[2][3])
        var validCount = 0
        val posErr_XYZ = floatArrayOf(dx,dy,dz)
        for (i in 0..2){
            if (abs(posErr_XYZ[i]) <= posTol){
                validCount +=1
            }
        }
        if (verbose) Log.i("IK_VERIFY", "posErr_XYZ = [${ posErr_XYZ[0]}, ${ posErr_XYZ[1]}, ${ posErr_XYZ[2]}]")
        if (validCount < 3) return false

        return true
    }

    fun verifyAllSolutions(
        sols: List<FloatArray>,
        T_target: Array<FloatArray>
    ) {
        var matchCount = 0

        if (verbose) Log.i("IK_VERIFY", "=== VERIFYING ${sols.size} IK SOLUTIONS ===")

        sols.forEachIndexed { i, sol ->
            if (verbose) {
                Log.i("IK_VERIFY", " ")
                Log.i("IK_VERIFY", "======= SOLUTION #$i =======")
            }
            val ok = verifySolution(sol, T_target)
            if (ok) matchCount++
        }
        if (verbose){
            Log.i("IK_VERIFY", "=== IK SUMMARY ===")
            Log.i("IK_VERIFY", "Matched solutions: $matchCount / ${sols.size}")
        }
    }

    fun getVerifiedSolutions(
        sols: List<FloatArray>,
        T_target: Array<FloatArray>
    ): List<FloatArray> {

        val sol_Veri = mutableListOf<FloatArray>()

        if (verbose) Log.i("IK_VERIFY", "================================ VERIFY ${sols.size} SOLUTIONS ===================================")

        sols.forEachIndexed { i, sol ->
            if (verbose) Log.i("IK_VERIFY", "Sol #$i: ${sol.joinToString { "%.2f°".format(it)}} ")
            val ok = verifySolution(sol, T_target)

            if (ok) {
                sol_Veri.add(sol)
                if (verbose) {
                    Log.i("IK_VERIFY", "VALID #$i  --> ${sol.joinToString { "%.4f°".format(it) }}")
                    Log.i("IK_VERIFY", "=======================================================================================")
                }

            }
            else {
                if (verbose) Log.i("IK_VERIFY", "=======================================================================================")
            }
        }
        if (verbose) {
            Log.i("IK_VERIFY", "=======================================================================================")
            Log.i("IK_VERIFY", "=== TOTAL SOLUTION(S) VERIFIED = ${sol_Veri.size} ===")
        }


        return sol_Veri
    }

    fun getVerified_Orient_Solutions(
        sols: List<FloatArray>,
        T_target: Array<FloatArray>
    ): List<FloatArray> {

        val sol_Veri = mutableListOf<FloatArray>()

        if (verbose) Log.i("IK_Orient__VERIFY", "================================ VERIFY ${sols.size} SOLUTIONS ===================================")

        sols.forEachIndexed { i, sol ->
            if (verbose) Log.i("IK_Orient__VERIFY", "Sol #$i: ${sol.joinToString { "%.2f°".format(it)}} ")
            val ok = verify_Orient_Solution(sol, T_target)

            if (ok) {
                sol_Veri.add(sol)
                if (verbose) {
                    Log.i("IK_Orient__VERIFY", "VALID #$i  --> ${sol.joinToString { "%.4f°".format(it) }}")
                    Log.i("IK_Orient__VERIFY", "=======================================================================================")
                }

            }
            else {
                if (verbose) Log.i("IK_VERIFY", "=======================================================================================")
            }
        }
        if (verbose) {
            Log.i("IK_Orient__VERIFY", "=======================================================================================")
            Log.i("IK_Orient__VERIFY", "=== TOTAL SOLUTION(S) VERIFIED = ${sol_Veri.size} ===")
        }


        return sol_Veri
    }



    fun eulerZYXToT(x: Float, y: Float, z: Float,
                    yawDeg: Float, pitchDeg: Float, rollDeg: Float): Array<FloatArray> {

        // Convert degrees to radians
        val yaw = Math.toRadians(yawDeg.toDouble()).toFloat()     // Z
        val pitch = Math.toRadians(pitchDeg.toDouble()).toFloat() // Y
        val roll = Math.toRadians(rollDeg.toDouble()).toFloat()   // X

        val cz = cos(yaw)
        val sz = sin(yaw)

        val cy = cos(pitch)
        val sy = sin(pitch)

        val cx = cos(roll)
        val sx = sin(roll)

        // Rotation matrix R = Rz * Ry * Rx   (ZYX)
        val r00 = roundIfTiny(cz * cy)
        val r01 = roundIfTiny(cz * sy * sx - sz * cx)
        val r02 = roundIfTiny(cz * sy * cx + sz * sx)

        val r10 = roundIfTiny(sz * cy)
        val r11 = roundIfTiny(sz * sy * sx + cz * cx)
        val r12 = roundIfTiny(sz * sy * cx - cz * sx)

        val r20 = roundIfTiny(-sy)
        val r21 = roundIfTiny(cy * sx)
        val r22 = roundIfTiny(cy * cx)

        // Transformation matrix T (4x4)
        return arrayOf(
            floatArrayOf(r00, r01, r02, x),
            floatArrayOf(r10, r11, r12, y),
            floatArrayOf(r20, r21, r22, z),
            floatArrayOf(0f, 0f, 0f, 1f)
        )
    }
}
