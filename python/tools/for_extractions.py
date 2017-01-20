#!/usr/bin/python
# vim: set fileencoding=utf-8

import re
from collections import namedtuple
from functools import partial
from for_clang import *

NTTypedef = namedtuple('typedef', ['name', 'fields', 'header'])
NTSBGLog = namedtuple('sbglog', ['var', 'type', 'enum'])


def extract_headers(source_lines,
                    regexp_for_include=r"#include \"sbgEComBinaryLog(.*).h\""):
    """
    Extraction des includes présents dans les sources d'un fichier (C).

    :param source_lines:
    :param regexp_for_include:
    :return:

    url: https://regexone.com/references/python

    >>> list(extract_headers(['#include "sbgEComBinaryLogDebug.h"']))
    ['Debug']

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


def extract_tuples_sbglog_types(tu,
                                func_filter=lambda n: True):
    """

    :param tu:
    :param func_filter:
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
    for node_from_root in iter_nodes(tu.cursor.walk_preorder(), [is_typedef_decl, func_filter]):
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


def extract_typedefs_from_headers(headers, index, list_options_for_clang):
    """

    :param headers:
    :param index:
    :param list_options_for_clang:
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
