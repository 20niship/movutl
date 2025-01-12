from pygen_types import MFunction, MEnum, MClass, MArgument
from typing import List


class SolWriter:
    PREFIX = "pygen_"

    STUB_COMMENT = "// Movutl autogenerated file(SolWriter)\n"

    def set(
        self,
        funcs_list: List[MFunction],
        enums_list: List[MEnum],
        classes_list: List[MClass],
    ):
        # for e in enums_list:
        #     self.register_enum(e)
        for c in classes_list:
            self.register_class(c)
        for f in funcs_list:
            self.register_func(f)
        # for m in members_list:
        #     self.register_property(m)

    def __init__(self, filename: str):
        self.autogen_text = (
            "#include <sol.hpp>\n"
            "#include <sol/sol.hpp>\n"
            "#include <sol/forward.hpp>\n"
            "#include <lua.hpp>\n"
            "namespace mu { \n"
            f"void generated_lua_binding_{self.PREFIX}(){{\n"
            "  sol::state_view lua;\n"
        )

        self.output_filename = "../movutl/generated/" + filename

    def save(self):
        self.autogen_text += "# ---- end of file ----"

        with open(self.output_filename, "w") as f:
            f.write(self.STUB_COMMENT)
            f.write(self.autogen_text)
            f.write("} // namespace mu\n")

    def register_func(self, func: MFunction):
        # sol::overload ?
        self.autogen_text += f'lua["{func.name}"] = &{func.name};\n'

    def register_class(self, cls: MClass):
        self.autogen_text += (
            (f"// {cls.desc}\n" if cls.desc else "")
            + f'lua.new_usertype<{cls.name}>("{cls.name}",\n'
            + "    sol::constructors<sol::types<>>(),\n"
        )

        for f in cls.funcs:
            self.autogen_text += f'  "{f.name}", &{cls.name}::{f.name},\n'
        for p in cls.props:
            self.autogen_text += f'  "{p.name}", &{cls.name}::{p.name},\n'
        self.autogen_text += ");\n"
