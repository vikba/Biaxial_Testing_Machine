from enum import Enum, auto


class SessionState(Enum):
    DISCONNECTED = auto()
    IDLE = auto()
    READY = auto()
    RUNNING_PRECONDITION = auto()
    AUTOLOADING = auto()
    RUNNING_TEST = auto()
    STOPPING = auto()
    ERROR = auto()
