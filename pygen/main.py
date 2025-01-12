import os
from typing import List
from pathlib import Path
import glob
from pygen_types import MFunction, MEnum, MClass, MArgument, ArgumentType
from SolWriter import SolWriter
from PropsWriter import PropsWriter
from config import ignore_symbols
from utils import logger, invalid_args, invalid_return_type

import CppHeaderParser

registered_classes: List[str] = []
registered_funcions: List = []
registered_enums: List[str] = []
skipped_nodes: List[CppHeaderParser.CppHeader] = []
saved_parent_classes: dict[str, List[str]] = {}


class Parser:
    def __init__(self):
        self.basestr = ""
        self.filename = ""
        self.module_stack: List[str] = []
        self.funcs: List[MFunction] = []
        self.classes: List[MClass] = []
        self.enums: List[MEnum] = []

    def _parse_class(self, node: CppHeaderParser.CppClass, nests: List[str]):
        classname: str = node["name"]
        if (classname, "") in ignore_symbols:
            return
        if classname[0] == "<" or "union" in classname:
            return
        if classname in registered_classes:
            return

        comment: str = node["doxygen"] if "doxygen" in node else ""

        member_funcs: List[MFunction] = []
        member_fields: List[MArgument] = []

        for method in node["methods"]["public"]:
            f = self._parse_function(method)
            if f is None or (classname, f.name) in ignore_symbols:
                continue
            f.constructor = f.name == classname
            f.destructor = f.name == "~" + classname
            member_funcs.append(f)

        for field in node["properties"]["public"]:
            name = field["name"]
            if (classname, name) in ignore_symbols:
                logger.warning(f"メンバー {name} は無視します")
                continue
            field_type = field["type"]
            comment = field["doxygen"] if "doxygen" in field else ""
            member_fields.append(
                MArgument(name, field_type, ArgumentType.ArgType_Undefined, comment)
            )

        if len(member_funcs) == 0 and len(member_fields) == 0:
            return

        if classname in registered_classes:
            return
        registered_classes.append(classname)

        c = MClass(classname, "", member_funcs, member_fields)
        logger.info(
            f"class: {classname} <-- Fx={len(member_funcs)}, Args={len(member_fields)}"
        )
        self.classes.append(c)

    #         if "enums" in node and "public" in node["enums"]:
    #             for enum in node["enums"]["public"]:
    #                 self._parse_enum(enum, classname_full)

    def _namespace_check(self, namespce: str) -> bool:
        if (
            "std" in namespce
            or "filemanager" in namespce
            or "melchior::detail" in namespce
            or "experimental" in namespce
            or "detail" in namespce
        ):
            return False
        return True

    def _parse_function(self, v: CppHeaderParser.CppMethod) -> MFunction | None:
        if v["name"] == ",":
            return None
        f = MFunction(v["name"])
        return_type = v["rtnType"]
        f.returns.c_type = return_type
        f.desc = v["doxygen"] if "doxygen" in v else ""
        f.is_static = v["static"]
        f.is_const = v["const"]
        args: List[MArgument] = []
        for arg in v["parameters"]:
            a = MArgument(arg["name"], arg["type"])
            a.detault_value = arg["default"] if "default" in arg else ""
            args.append(a)
        f.args = args

        if invalid_args(args):
            logger.warning(f"関数 {f.name} invalid ARGS : {args}")
            return None
        if invalid_return_type(return_type):
            logger.warning(f"関数 {f.name} invalid RETURN : {return_type}")
            return None

        if (f.name, len(args)) in registered_funcions:
            return None
        return f

    def _parse_functions(self, node: CppHeaderParser.CppHeader):
        for v in node.functions:
            namespce = v["namespace"]
            if not self._namespace_check(namespce):
                continue
            name = v["name"]
            if len(name) > 1 and name[0] == "_":
                continue
            if ("", name) in ignore_symbols:
                continue

            f = self._parse_function(v)
            if f == None:
                continue

            if (name, len(f.args)) in registered_funcions:
                return

            registered_funcions.append((name, len(f.args)))
            f.module_name = self.get_module_name()
            f.filename = self.filename
            self.funcs.append(f)

    def _parse_enums(self, node: CppHeaderParser.CppHeader):
        for enum in node.enums:
            namespce = enum["namespace"]
            if (
                namespce == "std"
                or "filemanager" in namespce
                or "melchior::detail" in namespce
                or "experimental" in namespce
            ):
                return
            self._parse_enum(enum, "")

    def _parse_enum(self, enum: CppHeaderParser.CppEnum, parent_str: str):
        if "name" not in enum:
            return

        enumname = enum["name"]
        if ("", enumname) in ignore_symbols:
            return
        if enumname == "" or enumname[0] == "_" or "(" in enumname or "/" in enumname:
            return
        if enumname in registered_enums:
            return
        registered_enums.append(enumname)

        comment = enum["doxygen"] if "doxygen" in enum else ""
        if parent_str != "":
            enumname = parent_str + "::" + enumname
        values = [v["name"] for v in enum["values"]]
        e = MEnum(enumname, comment, values)
        self.enums.append(e)

    def _parse_classes(self, node: CppHeaderParser.CppHeader):
        for _, v in node.classes.items():
            namespce = v["namespace"]
            if not self._namespace_check(namespce):
                continue
            self._parse_class(v, [])

    def parse(self, header_file: str):
        self.filename = header_file
        fstr = ""
        with open(header_file, "r") as f:
            fstr = f.read()
        delete_strs = [
            "final",
            "[[maybe_unused]]",
            "[[nodiscard]]",  #
            "final",
            "override",
            "inline",
            "constexpr",
            "noexcept",
            "explicit",  #
            "MU_API_FUNC",
            "MU_API",
            "IMGUI_API",
            "virtual",
            "explicit",
            "constexpr",
            "[[nodiscard]]",  #
        ]
        for d in delete_strs:
            fstr = fstr.replace(d, "")

        if "imgui" in header_file:
            strs_ = [
                """#ifndef IMGUI_DISABLE_OBSOLETE_FUNCTIONS
    ImGuiTreeNodeFlags_AllowItemOverlap     = ImGuiTreeNodeFlags_AllowOverlap,  // Renamed in 1.89.7
#endif
""",
                """
#ifndef IMGUI_DISABLE_OBSOLETE_FUNCTIONS
    ImGuiSelectableFlags_AllowItemOverlap   = ImGuiSelectableFlags_AllowOverlap,  // Renamed in 1.89.7
#endif
""",
                """
#ifndef IMGUI_DISABLE_OBSOLETE_FUNCTIONS
    ImGuiKey_ModCtrl = ImGuiMod_Ctrl, ImGuiKey_ModShift = ImGuiMod_Shift, ImGuiKey_ModAlt = ImGuiMod_Alt, ImGuiKey_ModSuper = ImGuiMod_Super, // Renamed in 1.89
    ImGuiKey_KeyPadEnter = ImGuiKey_KeypadEnter,    // Renamed in 1.87
#endif
""",
                """
#ifdef IMGUI_DISABLE_OBSOLETE_KEYIO
    ImGuiKey_KeysData_SIZE          = ImGuiKey_NamedKey_COUNT,  // Size of KeysData[]: only hold named keys
    ImGuiKey_KeysData_OFFSET        = ImGuiKey_NamedKey_BEGIN,  // Accesses to io.KeysData[] must use (key - ImGuiKey_KeysData_OFFSET) index.
#else
    ImGuiKey_KeysData_SIZE          = ImGuiKey_COUNT,           // Size of KeysData[]: hold legacy 0..512 keycodes + named keys
    ImGuiKey_KeysData_OFFSET        = 0,                        // Accesses to io.KeysData[] must use (key - ImGuiKey_KeysData_OFFSET) index.
#endif
""",
            ]
            for s in strs_:
                fstr = fstr.replace(s, "")

        if not os.path.exists(header_file):
            logger.error("Fileが存在しません！" + header_file)
            return

        header = CppHeaderParser.CppHeader(header_file)
        self._parse_functions(header)
        self._parse_enums(header)
        self._parse_classes(header)

        os.system(f"rm {header_file}")

    def get_module_name(self) -> str:
        return "_".join(self.module_stack)


def run():
    root = Path(__file__).parent.parent
    headers = [
        root / "movutl/asset/entity.hpp",
        root / "movutl/asset/movie.hpp",
        root / "movutl/asset/image.hpp",
        root / "movutl/asset/composition.hpp",
        root / "movutl/asset/project.hpp",
        root / "movutl/core/anim.hpp",
        root / "movutl/asset/text.hpp",
        root / "movutl/app/app.hpp",
        root / "movutl/gui/gui.hpp",
    ]

    logger.info(f"{len(headers)} headers found")

    last_generateds = glob.glob("./generated/*.cpp")
    for f in last_generateds:
        os.remove(f)

    funcs_list: List[MFunction] = []
    enums_list: List[MEnum] = []
    classes_list: List[MClass] = []

    for header in headers:
        p = Parser()
        p.parse(str(header))
        funcs_list.extend(p.funcs)
        enums_list.extend(p.enums)
        classes_list.extend(p.classes)

    funcs_list.sort(key=lambda x: x.name)
    enums_list.sort(key=lambda x: x.name)
    classes_list.sort(key=lambda x: x.name)

    stub_generater = SolWriter("generated_sol.cpp")
    stub_generater.set(funcs_list, enums_list, classes_list)
    stub_generater.save()

    stub_generater = PropsWriter("generated_props.cpp")
    stub_generater.set(funcs_list, enums_list, classes_list)
    stub_generater.save()


if __name__ == "__main__":
    run()
