from hexapod import Hexapod

def main():
    acp1 = Hexapod()
    acp1.set_leg_servo_channel(Hexapod.LEG1_IDX, Hexapod.COXA_IDX, 22)
    acp1.set_leg_servo_channel(Hexapod.LEG1_IDX, Hexapod.FEMUR_IDX, 23)
    acp1.set_leg_servo_channel(Hexapod.LEG1_IDX, Hexapod.TIBIA_IDX, 24)
    acp1.set_leg_servo_channel(Hexapod.LEG2_IDX, Hexapod.COXA_IDX, 18)
    acp1.set_leg_servo_channel(Hexapod.LEG2_IDX, Hexapod.FEMUR_IDX, 19)
    acp1.set_leg_servo_channel(Hexapod.LEG2_IDX, Hexapod.TIBIA_IDX, 20)
    acp1.set_leg_servo_channel(Hexapod.LEG3_IDX, Hexapod.COXA_IDX, 16)
    acp1.set_leg_servo_channel(Hexapod.LEG3_IDX, Hexapod.FEMUR_IDX, 15)
    acp1.set_leg_servo_channel(Hexapod.LEG3_IDX, Hexapod.TIBIA_IDX, 14)
    acp1.set_leg_servo_channel(Hexapod.LEG4_IDX, Hexapod.COXA_IDX, 11)
    acp1.set_leg_servo_channel(Hexapod.LEG4_IDX, Hexapod.FEMUR_IDX, 10)
    acp1.set_leg_servo_channel(Hexapod.LEG4_IDX, Hexapod.TIBIA_IDX, 9)
    acp1.set_leg_servo_channel(Hexapod.LEG5_IDX, Hexapod.COXA_IDX, 7)
    acp1.set_leg_servo_channel(Hexapod.LEG5_IDX, Hexapod.FEMUR_IDX, 6)
    acp1.set_leg_servo_channel(Hexapod.LEG5_IDX, Hexapod.TIBIA_IDX, 5)
    acp1.set_leg_servo_channel(Hexapod.LEG6_IDX, Hexapod.COXA_IDX, 3)
    acp1.set_leg_servo_channel(Hexapod.LEG6_IDX, Hexapod.FEMUR_IDX, 2)
    acp1.set_leg_servo_channel(Hexapod.LEG6_IDX, Hexapod.TIBIA_IDX, 1)

    acp1.set_led_fan_gpio(26)
    # 40 -- 140
    acp1.set_head_servo_channel(Hexapod.HEAD_X_IDX, 13)
    # 70 -- 105
    acp1.set_head_servo_channel(Hexapod.HEAD_Y_IDX, 12)

    acp1.reset_head()
    acp1.reset_all()
    acp1.loop()

if __name__ == '__main__':
    main()