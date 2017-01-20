#!/usr/bin/python
# vim: set fileencoding=utf-8
import os
import asciitree  # must be version 0.2
from mako.template import Template  # url: http://docs.makotemplates.org/en/latest/usage.html

from for_yaml import *
from for_clang import *
from for_extractions import *


def generate_rosmsg_attributes_from_typedef(typedef):
    """
    generator

    :param typedef:
    :return:
    :rtype: str
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
    print("Template(filename=%s, ...)" % input_filename)
    template = Template(filename=input_filename, input_encoding=input_encoding)
    gen_source_file = template.render(**data_for_mako)
    with open(output_filename, 'w') as fo:
        fo.write(gen_source_file.encode(output_encoding))
    print("output_filename: %s" % output_filename)


def generate_cpp_files(sbglogs, yaml_cfg):
    """

    :param sbglogs:
    :param yaml_cfg:
    :return:
    """
    datas_for_mako = {
        'sbglogs': sbglogs,
        'sbglogs_types': set(sbglog.type for sbglog in sbglogs)
    }

    cfg_mako = yaml_cfg['mako']
    import_directory = cfg_mako['import_directory']
    export_directory = cfg_mako['export_directory']
    templates_filenames = (
        (os.path.join(import_directory, template_import), os.path.join(export_directory, generated_export), prepath)
        for prepath, templates in walk_preorder(yaml_cfg['mako']['templates'])
        for template_import, generated_export in templates
    )

    for import_filename, export_filename, prepath in templates_filenames:
        print("* %s" % prepath)
        generate_cpp_file(import_filename, export_filename, **datas_for_mako)


def process(yaml_cfg):
    """

    :param yaml_cfg: YAML configuration
    """
    ############################################################
    cfg_clang = yaml_cfg['clang']
    cfg_clang_locations = cfg_clang['locations']
    # set the path to libclang library
    clang.cindex.Config.set_library_file(cfg_clang_locations['libclang'])
    index = clang.cindex.Index.create()
    # add compiler options
    list_options_for_clang = cfg_clang['options']
    # add includes paths
    list_options_for_clang.extend(
        ['-I{}'.format(os.path.abspath(include_path))
         for include_path in cfg_clang_locations['includes']]
    )
    ############################################################

    ####################################################################################################################
    # get the path to the first source code to analyze: sbgEComBinaryLogs.h
    ####################################################################################################################
    filename = cfg_clang_locations['sources']['sbgEComBinaryLogs']

    ############################################################
    with open(filename, 'r') as fo:
        sourcefile = fo.readlines()

    # Récupération de la liste des headers
    # regexp = r"#include \"sbgEComBinaryLog(.*).h\""
    headers = extract_headers(sourcefile)
    for header in headers:
        print(header + ".h")
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
    typedefs_from_headers = extract_typedefs_from_headers(headers, index, list_options_for_clang)
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

    ####################################################################################################################
    # get the path to the second source code to analyze: sbgEComIds.h
    ####################################################################################################################
    filename = cfg_clang_locations['sources']['sbgEComIds']

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
    ####################################################################################################################

    # Generate .h .cpp sources files from sbglogs analysis.
    generate_cpp_files(sbglogs_filtered, yaml_cfg)
