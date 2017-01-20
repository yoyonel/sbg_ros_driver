#!/usr/bin/python
# vim: set fileencoding=utf-8
import clang.cindex


def is_typedef_decl(n):
    """
    Test si le node transmis est node decrivant un déclaration de structure.

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.TYPEDEF_DECL


def is_enum_constant_decl(n):
    """
    Test si le node transmis est node decrivant une déclaration constante.

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.ENUM_CONSTANT_DECL


def is_union_decl(n):
    """
    Test si le node transmis est node decrivant une déclaration d'union.

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.UNION_DECL


def is_field_decl(n):
    """
    Test si le node transmis est node decrivant une champs de déclaration.

    :param n:
    :return:
    """

    return n.kind == clang.cindex.CursorKind.FIELD_DECL


def is_constantarray_type(n):
    """
    Test si le node transmis est node decrivant une déclaration de tableau constant.

    :param n:
    :return:
    """
    return n.type.kind == clang.cindex.TypeKind.CONSTANTARRAY


def is_integer_litteral(n):
    """
    Test si le node transmis est node decrivant une déclaration d'un litteral entier.

    :param n:
    :return:
    """
    return n.kind == clang.cindex.CursorKind.INTEGER_LITERAL


def node_children(node, filter=lambda n: True):
    """
    Filtre les noeuds enfants (children) avec une fonction de filtre transmise.

    :param node:
    :param filter:
    :return:
    """
    return (c for c in node.get_children() if filter(c))


def apply_filters(n, filters):
    """

    :param n:
    :param filters:
    :return:
    """
    return all(f(n) for f in filters)


def iter_nodes(nodes, funcs_filter):
    """

    :param nodes:
    :param funcs_filter:
    :return:
    """
    for n in nodes:
        if apply_filters(n, funcs_filter):
            yield n


def filter_source_filename(n, location_filename=""):
    """
    Fonction filtre d'un node par rapport au nom de son fichier source

    :param n:
    :param location_filename:
    :type location_filename: str
    :return:
    :rtype: bool
    """
    try:
        return n.location.file.name == location_filename if location_filename else True
    except:
        return False
    # return n.location.file.name == location_filename if location_filename else True


def node_to_string(node):
    """
    Stringification d'un noeud AST

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
