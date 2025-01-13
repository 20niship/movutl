from pygen_types import MFunction, MEnum, MClass, MArgument
from typing import List
from utils import write_if_different


class LuaIntfWriter:
    PREFIX = ""

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

    def __init__(self, filename: str):
        self.autogen_text = (
            "#include <LuaIntf/LuaIntf.h>\n"
            "#include <lua.hpp>\n"
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
            "namespace mu::detail { \n"
            f"void generated_lua_binding_{self.PREFIX}(Lua_State* L) {{\n"
            '    auto module = LuaBinding(L).beginModule("movutl");\n'
            "    module \\"
        )

        # LuaBinding(L)
        # .beginModule(string module_name)
        # .addFactory(function* func)
        # .addConstant(string constant_name, VALUE_TYPE value)
        # .addVariable(string property_name, VARIABLE_TYPE* var, bool writable = true)
        # .addVariableRef(string property_name, VARIABLE_TYPE* var, bool writable = true)
        # .addProperty(string property_name, FUNCTION_TYPE getter, FUNCTION_TYPE setter)
        # .addProperty(string property_name, FUNCTION_TYPE getter)
        # .addFunction(string function_name, FUNCTION_TYPE func)
        # .beginModule(string sub_module_name)
        #     ...
        # .endModule()

        # .beginClass<CXX_TYPE>(string class_name)
        #     ...
        # .endClass()

        # .beginExtendClass<CXX_TYPE, SUPER_CXX_TYPE>(string sub_class_name)
        #     ...
        # .endClass()
        # .endModule()

        # .beginExtendClass<CXX_TYPE, SUPER_CXX_TYPE>(string sub_class_name)
        # ...
        # .endClass()

        self.output_filename = "../movutl/generated/" + filename

    def save(self):
        output = (  #
            self.STUB_COMMENT  #
            + self.autogen_text  #
            + "   module.endModule();\n"  #
            + "}\n"  #
            + "} // namespace mu::detail\n"  #
        )
        write_if_different(self.output_filename, output)

    def register_func(self, func: MFunction):
        self.autogen_text += f'    .addFunction("{func.name}", &{func.name})\n'

    def register_class(self, cls: MClass):
        if cls.name == "Entity":
            return

        self.autogen_text += "  .beginClass<" + cls.name + '>("' + cls.name + '")\n'

        for f in cls.funcs:
            if cls.name == f.name:
                continue
            if "operator" in f.name:
                continue
            self.autogen_text += f'    .addFunction("{f.name}", &{cls.name}::{f.name})\n'
        for p in cls.props:
            self.autogen_text += f'    .addProperty("{p.name}", &{cls.name}::{p.name})\n'
        self.autogen_text += "  .endClass()\n"
