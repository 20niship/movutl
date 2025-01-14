from pygen_types import MFunction, MEnum, MClass, MArgument
from typing import List
from utils import write_if_different


class LuaIntfWriter:
    STUB_COMMENT = "// Movutl autogenerated file(LuaIntfWriter)\n"

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

    def __init__(self, filename: str, prefix: str = "movutl"):
        self.PREFIX = prefix
        self.autogen_text = (
            "#define LUAINTF_LINK_LUA_COMPILED_IN_CXX 0\n"
            "#include <LuaIntf/LuaIntf.h>\n"
            "#include <lua.hpp>\n"
            "#include <imgui.h>\n"
            "#include <movutl/core/props.hpp>\n"
            "#include <movutl/app/app.hpp>\n"
            "#include <movutl/plugin/input.hpp>\n"
            "#include <movutl/plugin/filter.hpp>\n"
            "#include <movutl/plugin/plugin.hpp>\n"
            "#include <movutl/asset/text.hpp>\n"
            "#include <movutl/asset/image.hpp>\n"
            "#include <movutl/asset/project.hpp>\n"
            "#include <movutl/asset/movie.hpp>\n"
            "#include <movutl/core/anim.hpp>\n"
            "#include <movutl/asset/track.hpp>\n"
            "#include <movutl/asset/entity.hpp>\n"
            "#include <movutl/asset/composition.hpp>\n"
            "extern \"C\" {\n" 
            "#include <lua.h>\n" 
            "#include <lauxlib.h>\n" 
            "#include <lualib.h>\n" 
            "}\n" 
            "\n"
            "namespace mu::detail { \n"
            "\n"
            "using namespace LuaIntf;\n"
            "using namespace ImGui;\n"
            "\n"
            f"void generated_lua_binding_{self.PREFIX}(lua_State* L) {{\n"
            f'    LuaBinding(L).beginModule("{self.PREFIX}")\n'
        )

        self.output_filename = "../movutl/generated/" + filename

    def save(self):
        output = (  #
            self.STUB_COMMENT  #
            + self.autogen_text  #
            + "   .endModule();\n"  #
            + "}\n"  #
            + "} // namespace mu::detail\n"  #
        )
        write_if_different(self.output_filename, output)

    def register_func(self, func: MFunction):
        self.autogen_text += f'    .addFunction("{func.name}", '
        self.autogen_text += f'static_cast<{func.returns.c_type}(*)( '
        for idx , a in enumerate(func.args):
            self.autogen_text += a.c_type
            if idx < len(func.args) - 1:
                self.autogen_text += ", "
        self.autogen_text += f')>(&{func.name}))\n'

    def register_class(self, cls: MClass):
        if cls.name == "Entity":
            return

        self.autogen_text += "  .beginClass<" + cls.name + '>("' + cls.name + '")\n'

        for f in cls.funcs:
            if cls.name == f.name:
                continue
            if "operator" in f.name:
                continue
            if f.is_static:
                self.autogen_text += f'    .addStaticFunction("{f.name}", &{cls.name}::{f.name})\n'
            else:
                self.autogen_text += f'    .addFunction("{f.name}", &{cls.name}::{f.name})\n'

        for p in cls.props:
            self.autogen_text += f'    .addVariable("{p.name}", &{cls.name}::{p.name}) // {p.c_type}\n'
        self.autogen_text += "  .endClass()\n"

