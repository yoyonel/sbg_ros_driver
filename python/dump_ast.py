#!/usr/bin/python
# vim: set fileencoding=utf-8
import clang.cindex
import asciitree  # must be version 0.2
import sys
import re


def node_children(node):
    return (c for c in node.get_children() if c.location.file.name == sys.argv[1])
    # return (c for c in node.get_children())


def print_node(node):
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


def extract_tuples_sbglog_types(tu):
    """

    :param tu:
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
        #                                          node.spelling, node.raw_comment))
        return str(node_type.spelling), str(node_level_field.raw_comment)

    def extract_from_union(node_level_typedef):
        """

        :param node_level_typedef:
        :return:
        """
        for node_from_union in node_level_typedef.get_children():
            if node_from_union.kind == clang.cindex.CursorKind.FIELD_DECL:
                l_sblog_types.append(extract_from_field(node_from_union))

    def extract_from_typedef(node_level_root):
        """

        :param node_level_root:
        :return:
        """
        for node_from_typedef in node_level_root.get_children():
            if node_from_typedef.kind == clang.cindex.CursorKind.UNION_DECL:
                extract_from_union(node_from_typedef)

    for node_from_root in tu.cursor.walk_preorder():
        if node_from_root.kind == clang.cindex.CursorKind.TYPEDEF_DECL \
                and node_from_root.location.file.name == filename:
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
        result = [prefix_for_enum + words_in_comment[0] for words_in_comment in comment_splitted]
    return result


def associate_sbglog_type_with_enums(sblog_types, prefix_for_enum='SBG_ECOM_LOG_'):
    """

    :param sblog_types:
    :type sblog_types: list
    :param prefix_for_enum:
    :type prefix_for_enum: str
    :return:
    :rtype: list

    """
    return [(sbglog_type, extract_enum_from_comment(comment))
            for sbglog_type, comment in sblog_types]


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: dump_ast.py [header file name]")
        sys.exit()

    filename = sys.argv[1]

    ####################################################################################################################
    with open(filename, 'r') as fo:
        sourcefile = fo.readlines()

    # Récupération de la liste des headers
    # regexp = r"#include \"sbgEComBinaryLog(.*).h\""
    headers = extract_headers(sourcefile)
    for header in headers:
        print(header + ".h")

    path_to_headers = "/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/src/binaryLogs/"

    ####################################################################################################################

    ####################################################################################################################
    # clang.cindex.Config.set_library_file('/usr/local/lib/libclang.so')
    clang.cindex.Config.set_library_file('/usr/lib/llvm-3.8/lib/libclang.so.1')
    index = clang.cindex.Index.create()
    # translation_unit = index.parse(sys.argv[1], ['-x', 'c++', '-std=c++11', '-D__CODE_GENERATOR__'])
    # translation_unit = index.parse(sys.argv[1], ['-x', 'c', '-D__CODE_GENERATOR__'])
    translation_unit = index.parse(
        filename,
        ['-Xclang', '-std=c', '-ast-dump', '-fsyntax-only', '-D__CODE_GENERATOR__',
         '-I/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/src',
         '-I/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/src/binaryLogs',
         '-I/home/atty/Prog/__IGN__/2015_LI3DS/__ROS__/Ellipse_N/ROS/overlay_ws/src/sbg_ros_driver/sbgECom/common',
         ]
    )

    # translation_unit.save(filename+'.ast')

    print(asciitree.draw_tree(translation_unit.cursor, node_children, print_node))
    # print(translation_unit.spelling)

    ###########################################
    # Récupération de la liste des headers
    ###########################################
    print("\n".join(list(extract_headers_from_ast(translation_unit))))

    print("")

    ###########################################
    # Association des types des sbglog avec les enums
    ###########################################
    sblog_types = extract_tuples_sbglog_types(translation_unit)
    # print("\n".join(", ".join(sblog_type) for sblog_type in sblog_types))

    for sbgtypes, enums in associate_sbglog_type_with_enums(sblog_types):
        print("{} -> {}".format(sbgtypes,
                                ", ".join(map(str, enums))))

    ####################################################################################################################
