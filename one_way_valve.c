#include "udf.h"

/* 单向阀 UDF
   用于 porous-jump 边界的 C2 系数
   正向 (vel_n > 0): C2 = 0   → 无阻力，自由通过
   反向 (vel_n < 0): C2 = 1e10 → 极大阻力，堵死 */

DEFINE_PROFILE(one_way_c2, thread, position)
{
    face_t f;
    real vel_n;
    Thread *t0, *t1 = NULL;
    cell_t c0, c1 = -1;
    real area[ND_ND];

    begin_f_loop(f, thread)
    {
        c0 = F_C0(f, thread);
        t0 = THREAD_T0(thread);
        c1 = F_C1(f, thread);
        t1 = THREAD_T1(thread);

        if (c1 >= 0 && t1 != NULL)
        {
            /* 面法向向量 */
            F_AREA(area, f, thread);

            /* 面法向平均速度 */
            vel_n = (C_U(c0, t0) + C_U(c1, t1)) * 0.5 * area[0] +
                    (C_V(c0, t0) + C_V(c1, t1)) * 0.5 * area[1] +
                    (C_W(c0, t0) + C_W(c1, t1)) * 0.5 * area[2];

            /* 正向: 无阻力; 反向: 堵死 */
            if (vel_n > 0.0)
                F_PROFILE(f, thread, position) = 0.0;
            else
                F_PROFILE(f, thread, position) = 1e10;
        }
        else
        {
            F_PROFILE(f, thread, position) = 0.0;
        }
    }
    end_f_loop(f, thread)
}
