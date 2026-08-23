from typing import List
from pygen_types import MArgument, ArgumentType, MAbiFunc
import os
import re


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


def should_autogen_func(line: str) -> bool:
    return "MUFUNC_AUTOGEN" in line


def parse_mprop_info(arg: MArgument, line: str) -> MArgument:
    if "[" in line and "]" in line and "[" not in arg.c_type:
        n = line[line.find("[") + 1 : line.find("]")]
        arg.c_type += "[" + n + "]"

    # 正規表現でkey=value形式をパース
    start = line.find("MPROPERTY(")
    if start == -1:
        return arg
    comment = line[start + len("MPROPERTY(") :]
    if comment[-1] == ")":
        comment = comment[:-1]

    for match in comment.split(","):
        kv = match.split("=")
        if len(kv) < 2:
            print(f"Invalid key-value pair: {match}")
            continue
        key = kv[0].strip()
        value = kv[1].strip()
        if key == "name":
            arg.dispname = value
        if key == "category":
            arg.category = value
        if key == "readonly":
            arg.readonly = value.lower() == "true"
        if key == "desc":
            arg.desc = value
        if key == "min":
            arg.minvalue = value
        if key == "max":
            arg.maxvalue = value
        if key == "step":
            arg.step = value
    arg.dispname = arg.dispname.replace('"', "")
    arg.category = arg.category.replace('"', "")
    arg.desc = arg.desc.replace('"', "")
    return arg


def get_prop_type(argtype: str) -> ArgumentType:
    argtype = argtype.replace("const ", "").replace(" ", "").replace("&", "")
    if argtype in [
        "uint8_t",
        "int32_t",
        "uint32_t",
        "unsignedint",
        "uint16_t",
        "int16_t",
        "uint64_t",
        "int64_t",
        "int",
    ]:
        return ArgumentType.ArgType_Int
    if argtype in ["float", "double"]:
        return ArgumentType.ArgType_Float
    if argtype == "bool":
        return ArgumentType.ArgType_Bool
    if argtype == "std::string":
        return ArgumentType.ArgType_String
    if argtype == "std::filesystem::path":
        return ArgumentType.ArgType_Path
    if argtype == "Vec3":
        return ArgumentType.ArgType_Vec3
    if argtype == "Vec2":
        return ArgumentType.ArgType_Vec2
    if argtype == "Vec4":
        return ArgumentType.ArgType_Vec4
    if argtype == "Vec4b":
        return ArgumentType.ArgType_Color
    if argtype == "Entity":
        return ArgumentType.ArgType_Entity
    # logger.error(f"Unsupported type: {argtype}")
    return ArgumentType.ArgType_Undefined


ABI_NAMESPACE_OPEN_RE = re.compile(r"^\s*namespace\s+(\w+)\s*\{")
ABI_FUNC_DECL_RE = re.compile(r"^\s*([\w:&*<>]+(?:\s+[\w:&*<>]+)*)\s+(\w+)\(([^()]*)\)\s*;\s*(//.*)?$")


def parse_abi_candidate_functions(header_file: str, exclude_names: set, include_path: str = "") -> List[MAbiFunc]:
    """`namespace detail { ... }` 直下の関数宣言をコメント不要で自動的にABI登録候補として拾う。
    exclude_names に含まれる関数名、関数ポインタを引数/戻り値に取るものは除外する。"""
    funcs: List[MAbiFunc] = []
    ns_stack: List[tuple] = []  # (name, depth_at_open)
    depth = 0
    with open(header_file, "r") as f:
        for line in f:
            stripped = line.strip()
            ns_match = ABI_NAMESPACE_OPEN_RE.match(stripped)
            in_detail = any(n == "detail" for n, _ in ns_stack)
            if ns_match:
                ns_stack.append((ns_match.group(1), depth))
                depth += 1
                continue
            if in_detail:
                m = ABI_FUNC_DECL_RE.match(line)
                if m:
                    ret_type, name, args = m.group(1).strip(), m.group(2).strip(), m.group(3).strip()
                    if name not in exclude_names and "(*" not in ret_type and "(*" not in args:
                        # abi_ プレフィックスは任意。あれば剥がしてフィールド名にするが、無くても関数名をそのまま使う
                        field_name = name[len("abi_"):] if name.startswith("abi_") else name
                        funcs.append(MAbiFunc(name=name, field_name=field_name, ret_type=ret_type, args=args, include_path=include_path))
            depth += stripped.count("{") - stripped.count("}")
            while ns_stack and depth <= ns_stack[-1][1]:
                ns_stack.pop()
    return funcs


def write_if_different(filename: str, output: str):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            if f.read() == output:
                return

    with open(filename, "w") as f:
        f.write(output)
