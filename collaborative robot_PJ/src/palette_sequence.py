import rclpy
import DR_init
import time
from collections import deque
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 100

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("doosan_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool, set_tcp, movel, wait,
            set_digital_output, set_singularity_handling,
            set_desired_force, release_force, release_compliance_ctrl,
            check_force_condition, move_periodic, check_position_condition, task_compliance_ctrl,
            DR_BASE, DR_TOOL, DR_AXIS_Z,
            DR_MV_MOD_REL, DR_MV_MOD_ABS,
            DR_FC_MOD_REL,
            get_current_posx
        )
        from DR_common2 import posj, posx

    except ImportError as e:
        print(f"Error importing Doosan API modules: {e}")
        return
                
# 변수 모음
    direction = 1
    row = 3
    col = 3
    stack = 1
    thickness = 0
    point_offset = [0,0,0]
    column = 3
    v = 200
    a = 50
    
    #Total count
    off_set_x = 0
    off_set_y = 0
    total_count = row * column * stack

    j = 0
    k = 3
    z = 6
    value_z = 0
    # 초기 세팅
    set_tool("Tool Weight_test")
    set_tcp("test_TCP")
    set_singularity_handling(ON)
    print("Speed and compliance setup completed")
    pos1 = posx(247.98, 99.0, 38.26, 0, 180, 0)
    basket = []
    basket.append(posx(248.00, -51, 140.00, 0, 180, 0))
    basket.append(posx(298.00, -51, 140.00, 0, 180, 0))
    basket.append(posx(348.00, -51, 140.00, 0, 180, 0))
    basket.append(posx(248.00, -101, 140.00, 0, 180, 0))
    basket.append(posx(298.00, -101, 140.00, 0, 180, 0))
    basket.append(posx(348.00, -101, 140.00, 0, 180, 0))
    basket.append(posx(248.00, -151, 140.00, 0, 180, 0))
    basket.append(posx(298.00, -151, 140.00, 0, 180, 0))
    basket.append(posx(348.00, -151, 140.00, 0, 180, 0))

    def grip():
        #  robot
        # set robot -> gripper
        # get gripper->robot check
        set_digital_output(1,1)
        set_digital_output(2,0)    
        wait(0.3)
        #wati(0.1)

    def release():
        set_digital_output(1,0)
        set_digital_output(2,1)    
        wait(0.3)

    def detecting():
        value_z = 0.0  # ← 반드시 초기화!
        while rclpy.ok():
            while not check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
                print("힘 체크 완료")
                release_force(time=0.0)
                release_compliance_ctrl()
                time.sleep(0.1)
                value_z = get_current_posx(ref=DR_BASE)[0][2]
                time.sleep(0.1)
                print(f"{value_z}")
                return value_z
        

    for i in range(0, total_count):
        # 제일 긴거 81,
        if i < 3 : 
            pos1[0] = pos1[0] + off_set_x
            
            movel(posx(247.98+off_set_x, 99.0, 38.26 + 80, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            grip()

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
            wait(0.5)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            value_z = detecting()
            movel(posx(247.98+off_set_x, 99.0, 100, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            release()
            movel(posx(0,0,-60,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            grip()
            movel(posx(0,0, 100,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

            if value_z > 80:
                movel(basket[j], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                wait(0.5)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                j = j +1
                off_set_x = off_set_x + 50
            elif value_z > 70:
                movel(basket[k], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                k = k + 1
                off_set_x = off_set_x + 50
            else:
                movel(basket[z], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                z = z + 1
                off_set_x = off_set_x + 50
        elif i < 6:
            if i == 3:
                off_set_x = 0
                off_set_y = 50
            pos1[0] = pos1[0] + off_set_x
            pos1[1] = pos1[1] + off_set_y
            
            movel(posx(247.98+off_set_x, 99.0 - off_set_y, 38.26 + 100, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            grip()

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
            wait(0.5)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            value_z = detecting()
            movel(posx(247.98+off_set_x, 99.0-off_set_y, 100, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            release()
            movel(posx(0,0,-60,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            grip()
            movel(posx(0,0, 100,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)  
            if value_z > 80:
                movel(basket[j], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                wait(0.5)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                j = j +1
                off_set_x = off_set_x + 50
            elif value_z > 70:
                movel(basket[k], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                k = k + 1
                off_set_x = off_set_x + 50
            else:
                movel(basket[z], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                z = z + 1
                off_set_x = off_set_x + 50     
        elif i < 9:
            if i == 6:
                off_set_x = 0
                off_set_y = 100
            pos1[0] = pos1[0] + off_set_x
            pos1[1] = pos1[1] + off_set_y
            
            movel(posx(247.98+off_set_x, 99.0 - off_set_y, 38.26 + 100, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            grip()

            task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
            # set_stiffnessx([3000.0]*3 + [200.0]*3, time=0.0)
            wait(0.5)
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            value_z = detecting()
            movel(posx(247.98+off_set_x, 99.0-off_set_y, 100, 0, 180, 0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            release()
            movel(posx(0,0,-60,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
            grip()
            movel(posx(0,0, 100,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)  
            if value_z > 80:
                movel(basket[j], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                wait(0.5)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                j = j +1
                off_set_x = off_set_x + 50
            elif value_z > 70:
                movel(basket[k], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=10, a=10 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)

                k = k + 1
                off_set_x = off_set_x + 50
            else:
                movel(basket[z], v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                movel(posx(0,0,-90,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                release()
                wait(3.0)
                movel(posx(0,0,90,0,0,0), v=20, a=20 ,vel=VELOCITY, acc=ACC, radius=0.0, ref=DR_BASE, mod=DR_MV_MOD_REL)
                z = z + 1
                off_set_x = off_set_x + 50 
    print("Pick-and-place sequence completed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
