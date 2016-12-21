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
from copy import deepcopy

NTTypedef = namedtuple('typedef', ['name', 'fields', 'header'])
NTSBGLog = namedtuple('sbglog', ['var', 'type', 'enum'])


def node_children(node, filter=lambda n: True):
    """

    :param node:
    :param filter:
    :return:
    """
    return (c for c in node.get_children() if filter(c))


def node_to_string(node):
    """

    :param node:
    :return:
    """
    text = node.spelling or node.displayname
    kind = str(node.kind)[str(node.kind).index('.') + 1:]

    if is_integer_litteral(node):
        return '{} = {}'.format(
            node.kind,
            node.get_tokens().next().spelling)

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
    try:
        return n.location.file.name == location_filename if location_filename else True
    except:
        return False


def is_typedef_decl(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.TYPEDEF_DECL


def is_enum_constant_decl(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.ENUM_CONSTANT_DECL


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


def is_integer_litteral(n):
    """

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.INTEGER_LITERAL


def extract_tuples_sbglog_types(_tu, _filter=lambda n: True):
    """

    :param _tu:
    :param _filter:
    :return:
    """
    # liste des associations enums/types des logs utilisés par SBG
    l_sblog_types = []

    def walk_into_field(node_field):
        """
        Forme un tuple 'NTSBGLog' depuis un 'field' déclaration.
        Création de ce tuple à partir des informations issues du noeuds:
        - nom de la variable
        - nom du type de la variable
        - commentaire associé à la variable

        :param node_field:
        """
        l_sblog_types.append(NTSBGLog(var=node_field.spelling,
                                      type=node_field.type.spelling,
                                      enum=node_field.raw_comment))

    def walk_into_union(node_union):
        """
        Recherche les 'fields' déclarations d'un union

        :param node_union:
        """
        for node_from_union in iter_nodes(node_union.get_children(), [is_field_decl]):
            walk_into_field(node_from_union)

    def walk_into_typedef(node_typedef):
        """
        Recherche les 'union' déclarations présents dans un typedef

        :param node_typedef:
        """
        for node_from_typedef in iter_nodes(node_typedef.get_children(), [is_union_decl]):
            walk_into_union(node_from_typedef)

    # Walking initial dans les noeuds
    # On parcourt depuis le root vers les noeuds 'typedef'
    # (c'est dans les noeuds 'typedef' qu'il y a les informations qui nous intéressent
    for node_from_root in iter_nodes(_tu.cursor.walk_preorder(), [is_typedef_decl, _filter]):
        walk_into_typedef(node_from_root)

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


def generate_rosmsg_attributes_from_typedef(typedef):
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


def generate_rosmsg_from_typedef(typedef, **kwargs):
    """

    :param typedef:
    :type typedef: NTTypedef
    :param kwargs:
    :return:
    """
    path_rosmsg = kwargs.get("path", "gen/msg/")
    ext_rosmsg = kwargs.get("ext", ".msg")
    #
    filename_rosmsg = path_rosmsg + typedef.name + ext_rosmsg

    with open(filename_rosmsg, 'w') as fo_ros_msg:
        fo_ros_msg.writelines("\n".join(generate_rosmsg_attributes_from_typedef(typedef)))


def generate_ros_msg_from_typedefs(typedefs, **kwargs):
    """

    :param typedefs:
    :param kwargs:
    :return:
    """
    for typedef in typedefs:
        generate_rosmsg_from_typedef(typedef, **kwargs)


def generate_cpp_file(input_filename,
                      output_filename,
                      input_encoding='utf-8',
                      output_encoding='utf-8',
                      **data_for_mako):
    """

    :param input_filename:
    :param output_filename:
    :param input_encoding:
    :param output_encoding:
    :param data_for_mako:
    :return:
    """
    template = Template(filename=input_filename, input_encoding=input_encoding)
    gen_source_file = template.render(**data_for_mako)
    with open(output_filename, 'w') as fo:
        fo.write(gen_source_file.encode(output_encoding))


def generate_cpp_files(sbglogs, **kwargs):
    """

    :param sbglogs:
    :param kwargs:
    :return:
    """
    datas_for_mako = {
        'sbglogs': sbglogs,
        'sbglogs_types': set(sbglog.type for sbglog in sbglogs)
    }

    templates_filenames = (
        ('mako/SBGLogtoROSMsg.h.txt', 'gen/mako/SBGLogtoROSMsg.h'),
        ('mako/SBGLogtoROSMsg.h.txt', 'gen/mako/SBGLogtoROSMsg.cpp'),
        ('mako/CMakeLists.txt.txt', 'gen/mako/CMakeLists.txt'),
        ('mako/SBGLogtoROSPublisher_Visitor.h.txt', 'gen/mako/SBGLogtoROSPublisher_Visitor.h'),
        ('mako/SBGLogtoROSPublisher_Visitor.cpp.txt', 'gen/mako/SBGLogtoROSPublisher_Visitor.cpp'),
        ('mako/wrapper_specialization.h.txt', 'gen/mako/wrapper_specialization.h'),
        ('mako/wrapper.h.txt', 'gen/mako/wrapper.h')
    )
    for import_filename, export_filename in templates_filenames:
        generate_cpp_file(import_filename, export_filename, **datas_for_mako)


def expand_sbglogs(sbglogs):
    """

    :param sbglogs:
    :return:
    """
    return (NTSBGLog(sbglog.var, sbglog.type, sbglog_enum)
            for sbglog in sbglogs
            for sbglog_enum in sbglog.enum)


if __name__ == "__main__":
    # clang.cindex.Config.set_library_file('/usr/local/lib/libclang.so')
    clang.cindex.Config.set_library_file('/usr/lib/llvm-3.8/lib/libclang.so.1')
    index = clang.cindex.Index.create()
    list_options_for_clang = [
        '-Xclang', '-std=c', '-ast-dump', '-fsyntax-only',
        '-D__CODE_GENERATOR__',
        '-I{}'.format(os.path.abspath('../sbgECom/src')),
        '-I{}'.format(os.path.abspath('../sbgECom/src/binaryLogs')),
        '-I{}'.format(os.path.abspath('../sbgECom/common')),
    ]

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
    ##########################################################################

    ##########################################################################
    translation_unit = index.parse(
        filename,
        list_options_for_clang
    )
    # translation_unit.save(filename+'.ast')

    bound_check_filename = partial(filter_source_filename, location_filename=filename)
    bound_node_children = partial(node_children, filter=bound_check_filename)
    print(asciitree.draw_tree(translation_unit.cursor,
                              bound_node_children,
                              node_to_string))
    # print(translation_unit.spelling)

    ###########################################
    # Association des types des sbglog avec les enums
    ###########################################
    sblog_types = extract_tuples_sbglog_types(translation_unit, bound_check_filename)

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
    # filtre les enums avec des '#' car pas encore traité ce cas
    sbglogs_filtered = filter(lambda sbglob: all('#' not in enum for enum in sbglob.enum), sbglogs)

    print("\n".join(str(sbglog) for sbglog in sbglogs_filtered))

    # Generate .h .cpp sources files from sbglogs analysis.
    # generate_cpp_files(sbglogs_filtered)

    #
    filename = '/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/src/sbgEComIds.h'
    tu = index.parse(
        filename,
        list_options_for_clang
    )
    #
    bound_check_filename = partial(filter_source_filename, location_filename=filename)
    bound_node_children = partial(node_children, filter=bound_check_filename)
    print(asciitree.draw_tree(tu.cursor,
                              bound_node_children,
                              node_to_string))
    #
    list_enums = list(n.spelling
                      for n in iter_nodes(tu.cursor.walk_preorder(),
                                          [bound_check_filename, is_enum_constant_decl]))
    #
    sbglogs_enums_contains_sharp = filter(lambda sbglob: not all('#' not in enum for enum in sbglob.enum), sbglogs)
    for sbglog in sbglogs_enums_contains_sharp:
        enums = sbglog.enum
        for enum in enums:
            if '#' in enum:
                regexp = enum.replace('#', '(.*)')
                found_enums = list(set(filter(lambda enum: re.search(regexp, enum), list_enums)))
                print("{} -> {}".format(enum, found_enums))
                #
                for new_enum in found_enums:
                    new_sbglog = sbglog._replace(enum=[new_enum])
                    # pour etendre la liste des types logs qu'on gère
                    sbglogs_filtered.append(new_sbglog)

    print("\n".join(map(str, sbglogs_filtered)))

    # Generate .h .cpp sources files from sbglogs analysis.
    generate_cpp_files(sbglogs_filtered)
