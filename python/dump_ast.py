#!/usr/bin/python
# vim: set fileencoding=utf-8
import clang.cindex
import asciitree  # must be version 0.2
import sys
import re
import os
from functools import partial
from collections import namedtuple
from mako.template import Template  # url: http://docs.makotemplates.org/en/latest/usage.html


NTTypedef = namedtuple('typedef', ['name', 'fields', 'header'])
NTSBGLog = namedtuple('sbglog', ['var', 'type', 'enum'])


def node_children(node, filter=lambda n: True):
    """

    :param node:
    :param filter:
    :return:
    """
    return (c for c in node.get_children() if filter(c))


def print_node(node):
    """

    :param node:
    :return:
    """
    text = node.spelling or node.displayname
    kind = str(node.kind)[str(node.kind).index('.') + 1:]

    # node.raw_comment

    try:
        return '[line={}, col={}] {} {} - {} = {} bytes'.format(
            node.location.line,
            node.location.column,
            kind,
            text,
            node.type.kind,
            node.type.get_size(),
        )
    except:
        return '{} {} [line={}, col={}]'.format(
            kind,
            text,
            node.location.line,
            node.location.column
        )


def extract_headers(source_lines,
                    regexp_for_include=r"#include \"sbgEComBinaryLog(.*).h\""):
    """

    :param source_lines:
    :param regexp_for_include:
    :return:

    url: https://regexone.com/references/python
    """
    return (re.findall(regexp_for_include, line)[0]
            for line in source_lines
            if re.search(regexp_for_include, line))


def extract_headers_from_ast(tu,
                             regexp_for_include=r"(.*)sbgEComBinaryLog(.*).h"):
    """

    :param tu:
    :param regexp_for_include:
    :return:
    """
    # include.location.file.name
    return (include.include.name
            for include in tu.get_includes()
            if include.depth == 1 and re.search(regexp_for_include, include.include.name))


def filter_source_filename(n, location_filename=""):
    """

    :param n:
    :param location_filename:
    :return:
    """
    return n.location.file.name == location_filename if location_filename else True


def is_typedef_decl(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.TYPEDEF_DECL


def is_union_decl(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.UNION_DECL


def is_field_decl(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.FIELD_DECL


def is_constantarray_type(n):
    """

    :param n:
    :return:
    """
    return n.type.kind == clang.cindex.TypeKind.CONSTANTARRAY


def extract_tuples_sbglog_types(tu, _filter=lambda n: True):
    """

    :param tu:
    :param _filter:
    :return:
    """
    l_sblog_types = []

    def extract_from_field(node_level_field):
        """

        :param node_level_field:
        :return:
        """
        node_type = node_level_field.type
        # print('\t\t{} [{} bytes] {} - {}'.format(node_type.spelling, node_type.get_size(),
        # node.spelling, node.raw_comment))
        # return str(node_type.spelling), str(node_level_field.raw_comment), str(node_level_field.spelling)
        return NTSBGLog(var=node_level_field.spelling,
                        type=node_type.spelling,
                        enum=node_level_field.raw_comment)

    def extract_from_union(node_level_typedef):
        """

        :param node_level_typedef:
        :return:
        """
        for node_from_union in iter_nodes(node_level_typedef.get_children(), [is_field_decl]):
            l_sblog_types.append(extract_from_field(node_from_union))

    def extract_from_typedef(node_level_root):
        """

        :param node_level_root:
        :return:
        """
        for node_from_typedef in iter_nodes(node_level_root.get_children(), [is_union_decl]):
            extract_from_union(node_from_typedef)

    for node_from_root in iter_nodes(tu.cursor.walk_preorder(), [is_typedef_decl, _filter]):
        extract_from_typedef(node_from_root)

    return l_sblog_types


def extract_enum_from_comment(comment, prefix_for_enum='SBG_ECOM_LOG_'):
    """

    :param comment:
    :param prefix_for_enum:
    :return:

    >>> comment = "/*!< Stores data for the SBG_ECOM_LOG_STATUS message. */"
    >>> extract_enum_from_comment(comment)
    ['SBG_ECOM_LOG_STATUS']

    >>> comment = "/*!< Stores data for the SBG_ECOM_LOG_SHIP_MOTION or SBG_ECOM_LOG_SHIP_MOTION_HP message. */"
    >>> extract_enum_from_comment(comment)
    ['SBG_ECOM_LOG_SHIP_MOTION', 'SBG_ECOM_LOG_SHIP_MOTION_HP']
    """
    result = []
    comment_splitted = comment.split(prefix_for_enum)[1:]
    if comment_splitted:
        comment_splitted = map(lambda s: s.split(" "), comment_splitted)
        result = [prefix_for_enum + words_in_comment[0]
                  for words_in_comment in comment_splitted]
    return result


def extract_enums_from_comments(_sblog_types, _prefix_for_enum='SBG_ECOM_LOG_'):
    """

    :param _sblog_types:
    :type _sblog_types: list
    :param _prefix_for_enum:
    :type _prefix_for_enum: str
    :return:
    :rtype: list

    """
    return [sbglog._replace(enum=extract_enum_from_comment(sbglog.enum, _prefix_for_enum))
            for sbglog in _sblog_types]


def apply_filters(n, _filters):
    """

    :param n:
    :param _filters:
    :return:
    """
    return all(f(n) for f in _filters)


def iter_nodes(nodes, funcs_filter):
    """

    :param nodes:
    :param funcs_filter:
    :return:
    """
    for n in nodes:
        if apply_filters(n, funcs_filter):
            yield n


def extract_typedefs_from_headers(headers):
    """

    :param headers:
    :type headers: list
    :return:
    """
    map_types = {
        'float': 'float32',
        'double': 'float64'
    }

    typedefs = []

    for header in headers:
        # print("header: {}".format(header))

        # translation unit from header source file parsing
        tu_header = index.parse(header, list_options_for_clang)

        # iter on typedef declaration
        bound_check_filename = partial(filter_source_filename, location_filename=header)
        for node_typedef in iter_nodes(tu_header.cursor.walk_preorder(), [is_typedef_decl, bound_check_filename]):
            # list fields for the typedef
            fields = []
            # iter on field declaration (inside typedef decl)
            for node_field in iter_nodes(node_typedef.walk_preorder(), [is_field_decl]):
                # get the type of the field decl
                type_field_ast = map_types.get(node_field.type.spelling, node_field.type.spelling)

                # if is a constant array
                if is_constantarray_type(node_field):
                    # get the type of element compose the array
                    type_element_cst_array = node_field.type.get_array_element_type().spelling
                    # format the field type with array element type and the size of the array
                    type_field_ast = "{}[{}]".format(
                        map_types.get(type_element_cst_array, type_element_cst_array),
                        node_field.type.get_array_size()
                    )

                # print("\tField decl: {} {}".format(
                #     type_field_ast,
                #     node_field.spelling or node_field.displayname)
                # )
                #
                fields.append((type_field_ast, node_field.spelling or node_field.displayname))

            if fields:
                # typedefs.append((node_typedef.spelling or node_typedef.displayname, fields))
                typedefs.append(NTTypedef(name=node_typedef.spelling, fields=fields, header=header))

    return typedefs


def generate_ros_msg_attributes_from_typedef(typedef):
    """

    :param typedef:
    :return:
    """
    # header
    yield "# Generated from: {}".format(typedef.header)
    yield "# {}".format(typedef.name)
    # list attributes
    for type_field, name_field in typedef.fields:
        str_attribute = "{} {}".format(type_field, name_field)
        yield str_attribute


def generate_ros_msg_from_typedef(typedef, **kwargs):
    """

    :param typedef:
    :type typedef: NTTypedef
    :param kwargs:
    :return:
    """
    path_rosmsg = kwargs.get("path", "gen/msg/")
    ext_rosmsg = kwargs.get("ext", ".msg")
    filename_rosmsg = path_rosmsg + typedef.name + ext_rosmsg

    with open(filename_rosmsg, 'w') as fo_ros_msg:
        fo_ros_msg.writelines("\n".join(generate_ros_msg_attributes_from_typedef(typedef)))


def generate_ros_msg_from_typedefs(typedefs, **kwargs):
    """

    :param typedefs:
    :param kwargs:
    :return:
    """
    for typedef in typedefs:
        generate_ros_msg_from_typedef(typedef, **kwargs)


def generate_cpp_files(sbglogs, **kwargs):
    """

    :param sbglogs:
    :param kwargs:
    :return:
    """
    sbglogs_types = set(sbglog.type for sbglog in sbglogs)

    # for ROS Message
    input_encoding = kwargs.get('input_encoding', 'utf-8')
    ouput_encoding = kwargs.get('output_encoding', 'utf-8')

    # SBGLogtoROSMsg.h
    filename_input = kwargs.get('filename_rosmsg_header_template', 'mako/SBGLogtoROSMsg.h.txt')
    filename_output = kwargs.get('filename_rosmsg_header_export', 'gen/mako/SBGLogtoROSMsg.h')
    template = Template(filename=filename_input, input_encoding=input_encoding)
    gen_source_file = template.render(sbglogs=sbglogs, sbglogs_types=sbglogs_types)
    with open(filename_output, 'w') as fo:
        fo.write(gen_source_file.encode(ouput_encoding))

    # SBGLogtoROSMsg.cpp
    filename_input = kwargs.get('filename_rosmsg_cpp_template', 'mako/SBGLogtoROSMsg.cpp.txt')
    filename_output = kwargs.get('filename_rosmsg_cpp_export', 'gen/mako/SBGLogtoROSMsg.cpp')
    template = Template(filename=filename_input, input_encoding=input_encoding)
    gen_source_file = template.render(sbglogs=sbglogs, sbglogs_types=sbglogs_types)
    with open(filename_output, 'w') as fo:
        fo.write(gen_source_file.encode(ouput_encoding))

    # CMakeLists.txt pour ROS (et les nouveaux messages)
    filename_input = kwargs.get('filename_cmakelists_template', 'mako/CMakeLists.txt.txt')
    filename_output = kwargs.get('filename_cmakelists_export', 'gen/mako/CMakeLists.txt')
    template = Template(filename=filename_input, input_encoding=input_encoding)
    gen_source_file = template.render(sbglogs=sbglogs, sbglogs_types=sbglogs_types)
    with open(filename_output, 'w') as fo:
        fo.write(gen_source_file.encode(ouput_encoding))

    # SBGLogtoROSPublisher_Visitor.h
    filename_input = kwargs.get('filename_rospub_visitor_header_template', 'mako/SBGLogtoROSPublisher_Visitor.h.txt')
    filename_output = kwargs.get('filename_rosmsg_header_export', 'gen/mako/SBGLogtoROSPublisher_Visitor.h')
    template = Template(filename=filename_input, input_encoding=input_encoding)
    gen_source_file = template.render(sbglogs=sbglogs, sbglogs_types=sbglogs_types)
    with open(filename_output, 'w') as fo:
        fo.write(gen_source_file.encode(ouput_encoding))

    # SBGLogtoROSPublisher_Visitor.cpp
    filename_input = kwargs.get('filename_rospub_visitor_cpp_template', 'mako/SBGLogtoROSPublisher_Visitor.cpp.txt')
    filename_output = kwargs.get('filename_rospub_visitor_cpp_template', 'gen/mako/SBGLogtoROSPublisher_Visitor.cpp')
    template = Template(filename=filename_input, input_encoding=input_encoding)
    gen_source_file = template.render(sbglogs=sbglogs, sbglogs_types=sbglogs_types)
    with open(filename_output, 'w') as fo:
        fo.write(gen_source_file.encode(ouput_encoding))


def expand_sbglogs(sbglogs):
    """

    :param sbglogs:
    :return:
    """
    return (NTSBGLog(sbglog.var, sbglog.type, enum) for sbglog in sbglogs for enum in sbglog.enum)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: dump_ast.py [header file name]")
        sys.exit()

    filename = sys.argv[1]

    ##########################################################################
    with open(filename, 'r') as fo:
        sourcefile = fo.readlines()

    # Récupération de la liste des headers
    # regexp = r"#include \"sbgEComBinaryLog(.*).h\""
    headers = extract_headers(sourcefile)
    for header in headers:
        print(header + ".h")

    path_to_headers = "/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/src/binaryLogs/"
    ##########################################################################

    ##########################################################################
    # clang.cindex.Config.set_library_file('/usr/local/lib/libclang.so')
    clang.cindex.Config.set_library_file('/usr/lib/llvm-3.8/lib/libclang.so.1')
    index = clang.cindex.Index.create()
    # translation_unit = index.parse(sys.argv[1], ['-x', 'c++', '-std=c++11', '-D__CODE_GENERATOR__'])
    # translation_unit = index.parse(sys.argv[1], ['-x', 'c', '-D__CODE_GENERATOR__'])

    list_options_for_clang = [
        '-Xclang', '-std=c', '-ast-dump', '-fsyntax-only',
        '-D__CODE_GENERATOR__',
        '-I{}'.format(os.path.abspath('../sbgECom/src')),
        '-I{}'.format(os.path.abspath('../sbgECom/src/binaryLogs')),
        '-I{}'.format(os.path.abspath('../sbgECom/common')),
    ]

    translation_unit = index.parse(
        filename,
        list_options_for_clang
    )

    # translation_unit.save(filename+'.ast')

    bound_check_filename = partial(filter_source_filename, location_filename=filename)
    bound_node_children = partial(node_children, filter=bound_check_filename)
    print(asciitree.draw_tree(translation_unit.cursor,
                              bound_node_children,
                              print_node))
    # print(translation_unit.spelling)

    ###########################################
    # Association des types des sbglog avec les enums
    ###########################################
    sblog_types = extract_tuples_sbglog_types(translation_unit, bound_check_filename)
    # print("\n".join(", ".join(sblog_type) for sblog_type in sblog_types))

    # for sbglog_type, enums, sbglog_var in associate_sbglog_type_with_enums(sblog_types):
    sbglogs = extract_enums_from_comments(sblog_types)
    for sbglog_var, sbglog_type, enums in sbglogs:
        print("{} ({}) -> {}".format(
            sbglog_type,
            sbglog_var,
            ", ".join(map(str, enums)))
        )

    ###########################################
    # Récupération de la liste des headers
    ###########################################
    headers = list(extract_headers_from_ast(translation_unit))
    # print("\n".join(headers))
    # print("")

    print("Extract typedefs from {} headers files ...".format(len(headers)))
    typedefs_from_headers = extract_typedefs_from_headers(headers)
    print("-> {} typedef declarations found.".format(len(typedefs_from_headers)))

    generate_ros_msg_from_typedefs(typedefs_from_headers, path="gen/msg/")

    # for name_typedef, fields in typedefs_from_headers:
    #     print("{}".format(name_typedef))
    #     for type_field, name_field in fields:
    #         print("\t{} {}".format(type_field, name_field))

    # bound_node_children = partial(node_children, filter=bound_check_filename)
    # print(asciitree.draw_tree(tu_header.cursor,
    #                           bound_node_children,
    #                           print_node))
    ##########################################################################
    # sbglogs_filtered = filter(lambda sbglob: '#' not in sbglob.enum, expand_sbglogs(sbglogs))
    sbglogs_filtered = filter(lambda sbglob: all('#' not in enum for enum in sbglob.enum), sbglogs)
    print("\n".join(str(sbglog) for sbglog in sbglogs_filtered))
    generate_cpp_files(sbglogs_filtered)
