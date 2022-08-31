from PyCM730 import *
import time
import rclpy

def main():
    cm730 = CM730()
    cm730.connect()
    cm730.dxl_on()
    time.sleep(1)

    try:
      cm730.check_ID(0, 255)
    except:
        pass
    cm730.servo_sync_disable_torque(list(range(0,20)))
    #cm730.servo_sync_enable_torque([18, 21])

    tick = time.time()
    dt_all = 0.0
    dt_i = 0
    while True:
    #for i in range(0, 1000, 10):
#        cm730.servo_sync_write_position([18, 21], [i, i])
        cm730.servo_bulk_read_position(list(range(1,20)))
        #print(cm730.servo_bulk_read_position([18, 20]))
        time.sleep(0.01)
        tock = time.time()
        dt = tock - tick
        tick = tock
        dt_all = dt_all + dt
        dt_i = dt_i + 1

        if (dt_i > 100):
            dt_f = dt_all / 100.0
            print(dt_f)
            dt_all = 0.0
            dt_i = 0

    cm730.servo_sync_disable_torque([18, 21])
    cm730.disconnect()
    
if __name__ == "__main__":
    main()
