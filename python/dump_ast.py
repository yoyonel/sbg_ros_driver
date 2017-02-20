#!/usr/bin/python
# vim: set fileencoding=utf-8
from tools import *
import tools.clang_ast as tca

if __name__ == "__main__":
    # ca = tca.ClangAST()
    ca = tca.ClangAST(yaml_setting_filename="settings/settings.yml")
    ca.process()
    # process(configurations_from_yaml("settings/settings.yml"))
#

