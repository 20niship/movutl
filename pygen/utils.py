from typing import List
from pygen_types import MArgument, MFunction, MEnum, MClass


class ColoredLogger:
    green_ = "\033[32m"
    yellow_ = "\033[33m"
    red_ = "\033[31m"
    end_ = "\033[0m"

    def __init__(self):
        pass

    def info(self, msg: str):
        print(f"{self.green_}{msg}{self.end_}")

    def warning(self, msg: str):
        print(f"{self.yellow_}{msg}{self.end_}")

    def error(self, msg: str):
        print(f"{self.red_}{msg}{self.end_}")


logger = ColoredLogger()


def cvt_type(t: str) -> str:
    if t == "float":
        return "ArgType_Float"
    if t == "int":
        return "ArgType_Int"
    if t == "bool":
        return "ArgType_Bool"
    if t == "string":
        return "ArgType_String"
    if t == "path":
        return "ArgType_Path"
    if t == "selection":
        return "ArgType_Selection"
    if t == "vec3":
        return "ArgType_Vec3"
    if t == "vec4":
        return "ArgType_Vec4"
    if t == "entity":
        return "ArgType_Entity"
    if t == "color":
        return "ArgType_Color"
    return "ArgType_Undefined"


def invalid_args(args: List[MArgument]):
    return False


def invalid_return_type(ret: MArgument):
    return False
