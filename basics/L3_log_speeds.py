
import L1_log as log
import L2_kinematics as kin
from time import sleep

sleep(5)

while True:
    C = kin.getMotion()  # This take approx 25.1 ms if the delay is 20ms
    log.tmpFile(C[0],"xdote")
    log.tmpFile(C[1],"tdote")
    print("xdot(m/s), thetadot (rad/s):", C)

    wheel_speeds = kin.getPdCurrent()
    log.tmpFile(wheel_speeds[0],"pdl")
    log.tmpFile(wheel_speeds[1],"pdr")
    print("test:",wheel_speeds)

    sleep(0.25)