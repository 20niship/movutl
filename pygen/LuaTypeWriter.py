from pygen_types import MFunction, MEnum, MClass, MArgument
from typing import List
from utils import write_if_different

def lua_cvt_type(ctype) -> str:
    if ctype == "S":
        return "string"
    if "const char" in ctype:
        return "string"

    delete_strs = ["const ", "&", "*", "static ", "virtual ", "const ", "unsigned " ]
    for s in delete_strs:
        ctype = ctype.replace(s, "")

    if "int" in ctype:
        return "number"
    if "std::vector" in ctype:
        return "table"

    match ctype:
        case "float":
            return "number"
        case "void":
            return "nil"
        case "double":
            return "number"
        case "bool":
            return "boolean"
        case "std::string":
            return "string"
    return ctype

def get_default(arg: MArgument) -> str:
    ctype = lua_cvt_type(arg.c_type)
    match ctype:
        case "number":
            return "0"
        case "string":
            return '""'
        case "boolean":
            return "false"
        case "table":
            return "{}"
        case "nil":
            return "nil"
    if ctype in ["ImVec2", "ImVec4", "ImColor", "ImTextureID", "Vec2", "Vec3", "Vec4"]:
        return f"{ctype}()"
    return "nil"

default_types = """
---@class ImVec2
---@field x number
---@field y number
imgui.ImVec2= {}

---@class ImVec4
---@field x number
---@field y number
---@field z number
---@field w number
imgui.ImVec4= {}

---@class ImColor
---@field Value ImVec4
imgui.ImColor= {}

"""

class LuaTypeWriter:
    STUB_COMMENT = "-- movutl pygen auto generated bindings\n"

    def set(
        self,
        funcs_list: List[MFunction],
        enums_list: List[MEnum],
        classes_list: List[MClass],
    ):
        for e in enums_list:
            self.register_enum(e)
        for c in classes_list:
            if c.name in ["ImVec2", "ImVec4", "ImColor"]:
                continue
            self.register_class(c)
        for f in funcs_list:
            self.register_func(f)
        # for m in members_list:
        #     self.register_property(m)

    def __init__(self, filename: str, prefix: str = "movutl"):
        self.PREFIX = prefix
        self.autogen_text = (
                "--@meta\n"
                f"--@class {self.PREFIX}\n"
                f"local {self.PREFIX} = {{}}\n"
        )
        self.autogen_text += default_types
        self.output_filename = "../lancher/runtime/" + filename

    def save(self):
        output = (  #
            self.STUB_COMMENT  #
            + self.autogen_text  #
            + "   return " + self.PREFIX  #
        )
        write_if_different(self.output_filename, output)

    def register_func(self, func: MFunction):
        fname = func.name
        if "ImGui" in func.namespace and fname.endswith("_"):
            fname = fname[:-1]
        for a in func.args:
            self.autogen_text += f'---@param {a.name} {lua_cvt_type(a.c_type)}\n'
        self.autogen_text += f'---@return {lua_cvt_type(func.returns.c_type)}\n'
        self.autogen_text += f'function {self.PREFIX}.{fname}( '
        for a in func.args:
            self.autogen_text += a.name + ", " 
        self.autogen_text += ")end\n\n"

    def register_enum(self, enum: MEnum):
        self.autogen_text += f"---@class {enum.name}\n"
        for e in enum.values:
            self.autogen_text += f'---@field {e} number\n'
        self.autogen_text += f"{self.PREFIX}.{enum.name} = {{}}\n\n"

    def register_class(self, cls: MClass):
        if cls.name == "Entity":
            return

        self.autogen_text += f"---@class {cls.name}\n"
        for p in cls.props:
            self.autogen_text += f'---@field {p.name} {lua_cvt_type(p.c_type)}\n'
        self.autogen_text += f"{self.PREFIX}.{cls.name} = {{}}\n"
        for p in cls.props:
            default = p.detault if p.detault else get_default(p)
            self.autogen_text += f'{self.PREFIX}.{cls.name}.{p.name} = {default}\n'

        for f in cls.funcs:
            if cls.name == f.name:
                continue
            if f.name.startswith("operator"):
                continue
            self.autogen_text += "\n"
            for a in f.args:
                self.autogen_text += f'---@param {a.name} {lua_cvt_type(a.c_type)}\n'
            self.autogen_text += f'---@return {lua_cvt_type(f.returns.c_type)}\n'
            self.autogen_text += f'function {self.PREFIX}.{cls.name}:{f.name}( '
            for a in f.args:
                self.autogen_text += a.name + ", "
            self.autogen_text += ") end\n"

        self.autogen_text += "\n"

