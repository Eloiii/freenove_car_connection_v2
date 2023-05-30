from enum import Enum


class Command(Enum):
    CMD_MOTOR = 'motor'
    CMD_SERVO = 'servo'
    CMD_LED = 'led'
    CMD_SONIC = 'sonic'
    CMD_BUZZER = 'buzzer'
    CMD_LIGHT = 'light'
    CMD_DATA = 'data'
