from dataclasses import dataclass
from dataclasses import field
from typing import List
from enum import Enum


class ArgumentType(Enum):
    ArgType_Float = 1
    ArgType_Int = 2
    ArgType_Bool = 3
    ArgType_String = 4
    ArgType_Path = 5
    ArgType_Selection = 6
    ArgType_Vec2 = 7
    ArgType_Vec3 = 8
    ArgType_Vec4 = 9
    ArgType_Entity = 10
    ArgType_Color = 11
    ArgType_Undefined = 12


@dataclass
class MArgument:
    name: str = ""
    dispname: str = ""
    c_type: str = ""
    ptype: ArgumentType = ArgumentType.ArgType_Undefined
    detault: str = ""
    category: str = ""
    desc: str = ""
    is_angle: bool = False
    readonly: bool = False
    minvalue: str = ""
    maxvalue: str = ""
    namespace :str = ""
    step: str = ""

    def valid(self) -> bool:
        if self.c_type in ["void", "void *"]:
            return False
        if "*" in self.c_type and "const char *" not in self.c_type:
            return False
        if "[" in self.c_type and "]" in self.c_type:
            return False
        return True


@dataclass
class MFunction:
    name: str
    returns: MArgument = field(default_factory=MArgument)
    args: List[MArgument] = field(default_factory=list)
    desc: str = ""
    should_autogen: bool = False
    is_static: bool = False
    is_const: bool = False
    constructor: bool = False
    namespace :str = ""
    destructor: bool = False
    filename :str = ""

    def valid_args(self) -> bool:
        for a in self.args:
            if not a.valid():
                return False
        return True

    def valid_returns(self) -> bool:
        return "*" not in self.returns.name

@dataclass
class MEnum:
    name: str
    desc: str
    values: List[str]


@dataclass
class MClass:
    name: str
    desc: str
    funcs: List[MFunction]
    props: List[MArgument]
    namespace :str = ""
    filename :str = ""

    def has_multi_funcs(self, fname: str) -> bool:
        return len([f for f in self.funcs if f.name == fname]) > 1
