import numpy as np

def func_G(g,m2,m3,m4,m5,m6,q2,q3,q4,q5,q6):
    t2 = np.sin(q2)
    t3 = np.cos(q2)
    t4 = np.cos(q3)
    t5 = np.sin(q3)
    t6 = np.cos(q4)
    t7 = np.sin(q4)
    t8 = np.cos(q5)
    t9 = np.sin(q5)
    t10 = np.sin(q6)
    t11 = np.cos(q6)
    t12 = m3*t3*t4*5.2e2
    t13 = m4*t3*t4*3.435e3
    t14 = m5*t3*t4*3.8e3
    t15 = m6*t3*t4*3.8e3
    t16 = m6*t3*t4*t8*6.05e2
    t17 = m4*t3*t5*t6
    t18 = m4*t2*t4*t6
    t19 = m5*t3*t4*t9
    t20 = m5*t3*t5*t6*t8
    t21 = m5*t2*t4*t6*t8
    t22 = m6*t3*t4*t9*t11
    t23 = m6*t3*t5*t6*t8*t11
    t24 = m6*t2*t4*t6*t8*t11
    t25 = q2+q3
    t26 = np.cos(t25)
    t27 = np.sin(t25)
    G = np.array([0.0,g*(t12+t13+t14+t15+t16+t17+t18+t19+t20+t21+t22+t23+t24+m2*t2*1.896e3-m2*t3*8.8e1+m3*t2*3.6e3+m4*t2*3.6e3+m5*t2*3.6e3+m6*t2*3.6e3-m3*t2*t4*2.9e1-m3*t2*t5*5.2e2-m3*t3*t5*2.9e1-m4*t2*t5*3.435e3-m5*t2*t5*3.8e3-m6*t2*t5*3.8e3-m4*t2*t4*t7*5.0-m4*t3*t5*t7*5.0-m5*t2*t5*t9-m6*t2*t5*t8*6.05e2-m6*t2*t4*t6*t9*6.05e2-m6*t2*t4*t7*t10-m6*t3*t5*t6*t9*6.05e2-m6*t3*t5*t7*t10-m6*t2*t5*t9*t11)*(-1.0e-4),g*(-t12-t13-t14-t15-t16-t17-t18-t19-t20-t21-t22-t23-t24+m3*t2*t4*2.9e1+m3*t2*t5*5.2e2+m3*t3*t5*2.9e1+m4*t2*t5*3.435e3+m5*t2*t5*3.8e3+m6*t2*t5*3.8e3+m4*t2*t4*t7*5.0+m4*t3*t5*t7*5.0+m5*t2*t5*t9+m6*t2*t5*t8*6.05e2+m6*t2*t4*t6*t9*6.05e2+m6*t2*t4*t7*t10+m6*t3*t5*t6*t9*6.05e2+m6*t3*t5*t7*t10+m6*t2*t5*t9*t11)*1.0e-4,g*t26*(m4*t6*5.0+m4*t7+m5*t7*t8+m6*t6*t10-m6*t7*t9*6.05e2+m6*t7*t8*t11)*(-1.0e-4),-g*m6*(t9*t27*(-6.05e-2)+t6*t8*t26*6.05e-2+t8*t11*t27*1.0e-4+t6*t9*t11*t26*1.0e-4)-g*m5*(t8*t27*1.0e-4+t6*t9*t26*1.0e-4),-g*m6*(t7*t11*t26*1.0e-4-t9*t10*t27*1.0e-4+t6*t8*t10*t26*1.0e-4)])
    return G