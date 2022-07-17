

class LegJointsEnum:
    """
    Enum for the leg joints.
    """
    FL_COXA = 13
    FL_FEMUR = 12
    FL_TIBIA = 9
    FR_COXA = 2
    FR_FEMUR = 0
    FR_TIBIA = 1
    BL_COXA = 11
    BL_FEMUR = 3
    BL_TIBIA = 8
    BR_COXA = 14
    BR_FEMUR = 15
    BR_TIBIA = 4
    HEAD_YAW = 10
    HEAD_PITCH = 5

    def __iter__(self):
        return (i for i in range(16))



HOME_POS = [0. for i in range(16)]
HOME_POS[LegJointsEnum.FL_COXA] = 0.3
HOME_POS[LegJointsEnum.FL_FEMUR] = 1.0
HOME_POS[LegJointsEnum.FL_TIBIA] = 0.75
HOME_POS[LegJointsEnum.FR_COXA] = 0.7
HOME_POS[LegJointsEnum.FR_FEMUR] = 0.0
HOME_POS[LegJointsEnum.FR_TIBIA] = 0.25
HOME_POS[LegJointsEnum.BL_COXA] = 0.7
HOME_POS[LegJointsEnum.BL_FEMUR] = 0.0
HOME_POS[LegJointsEnum.BL_TIBIA] = 0.25
HOME_POS[LegJointsEnum.BR_COXA] = 0.3
HOME_POS[LegJointsEnum.BR_FEMUR] = 1.0
HOME_POS[LegJointsEnum.BR_TIBIA] = 0.75
HOME_POS[LegJointsEnum.HEAD_YAW] = 0.5
HOME_POS[LegJointsEnum.HEAD_PITCH] = 0.6



SERVO_MOVE_TIME_MS = 2500

