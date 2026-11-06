'''
Parameter override files given to every node through the param_overrides launch argument.

The argument is a space-separated list of YAML parameter files. They are merged in order,
a later file winning over an earlier one, and the result is passed last to each node,
after its own parameters.

rcl applies parameter sections in the order their keys first appear, not file by file: a
/**/<node> section of an override would land in the package file's earlier section, before the
section launch_ros writes for the parameters a launch file gives. So a named node gets its
overrides keyed by its full name, which comes after both. A node with no name in the launch
file, and a component, get them keyed '/**/**' and '/**/**/<node>': these match the same nodes
as '/**' and '/**/<node>', but no other file uses them, so they also come last.
'''

import copy
import os
import tempfile
from typing import Any, Callable, List, Optional, Text

import yaml
from launch import Substitution
from launch.launch_context import LaunchContext
from launch.logging import get_logger, launch_config
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

ARG_NAME = 'param_overrides'


def _is_wildcard_key(key) -> bool:
    if key == '/**':
        return True
    name = key[len('/**/'):] if isinstance(key, str) and key.startswith('/**/') else ''
    return bool(name) and '/' not in name


def _deep_merge(base: dict, update: dict) -> dict:
    '''
    Merges update into base: nested dictionaries are merged, any other value is replaced
    '''
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def read_yaml_mapping(path: Text, arg_name: Text) -> dict:
    '''
    Returns the content of the YAML file given by a launch argument, empty for an empty file.
    Raises RuntimeError on a missing file or one that is not a mapping.
    '''
    if not os.path.isfile(path):
        raise RuntimeError(f'{arg_name}: file not found: {path}')
    with open(path) as f:
        content = yaml.safe_load(f) or {}
    if not isinstance(content, dict):
        raise RuntimeError(f'{arg_name}: {path} is not a YAML mapping')
    return content


def merge_override_files(paths: List[Text]) -> dict:
    '''
    Loads the given parameter files and merges them in order, the last one winning.
    Every top-level key must be '/**' or '/**/<node>': two spellings of the same node
    would stay separate sections whose relative order is not defined.
    Raises RuntimeError on a missing file, a file that is not a mapping or a wrong key.
    '''
    merged = {}
    for path in paths:
        content = read_yaml_mapping(path, ARG_NAME)
        wrong = [key for key in content if not _is_wildcard_key(key)]
        if wrong:
            raise RuntimeError(
                f'{ARG_NAME}: {path} has keys {wrong}, key node sections as "/**/<node>"')
        _deep_merge(merged, content)
    return merged


def node_sections(merged: dict, full_name: Optional[Text] = None) -> dict:
    '''
    Returns the parameter file content of one node. With its full name, the '/**' and
    '/**/<node>' sections are merged under that name; without, every section is kept under
    its '/**/**' spelling, the '/**' one first so that the node sections win over it.
    '''
    if full_name is None:
        keys = sorted(merged, key=lambda key: key != '/**')
        return {'/**/**' + key[len('/**'):]: merged[key] for key in keys} \
            or {'/**/**': {'ros__parameters': {}}}
    params = {}
    for section in ('/**', '/**/' + full_name.rsplit('/', 1)[-1]):
        _deep_merge(params, copy.deepcopy(merged.get(section, {}).get('ros__parameters', {})))
    return {full_name: {'ros__parameters': params}}


def write_overrides(content: dict, paths: List[Text]) -> Text:
    '''
    Writes a parameter file headed by the override files it comes from to the launch log
    directory, and returns its path
    '''
    os.makedirs(launch_config.log_dir, exist_ok=True)
    fd, out_path = tempfile.mkstemp(prefix=f'{ARG_NAME}_', suffix='.yaml', dir=launch_config.log_dir)
    with os.fdopen(fd, 'w') as f:
        f.write(''.join(f'# {path}\n' for path in paths))
        yaml.safe_dump(content, f, default_flow_style=False)
    return out_path


def full_node_name(node, unnamed: Optional[Text] = None) -> Optional[Text]:
    '''
    Returns the full name of a launch_ros Node whose substitutions are performed. A node given
    no name takes the unnamed one instead, or has no full name when that is None.
    '''
    name = node.node_name
    if node.UNSPECIFIED_NODE_NAME in name:
        if unnamed is None:
            return None
        name = name.replace(node.UNSPECIFIED_NODE_NAME, unnamed)
    return '/' + name.replace(node.UNSPECIFIED_NODE_NAMESPACE, '').lstrip('/')


def launch_global(context: LaunchContext, key: Text, create: Callable[[], Any]) -> Any:
    '''
    Returns a value kept for the whole launch in the globals of its context, made by create
    the first time it is asked for
    '''
    value = context.get_locals_as_dict().get(key)
    if value is None:
        value = create()
        context.extend_globals({key: value})
    return value


def launch_overrides(context: LaunchContext, paths: List[Text]) -> dict:
    '''
    Returns the merged override files, read once per launch and kept on its context
    '''
    cache = launch_global(context, 'simple_launch_overrides', dict)
    key = tuple(paths)
    if key not in cache:
        cache[key] = merge_override_files(paths)
        if paths:
            get_logger('simple_launch').info(f'{ARG_NAME}: {" ".join(paths)}')
    return cache[key]


class MergedOverrides(Substitution):
    '''
    Substitution that resolves to the override file of one node, from the param_overrides
    launch argument
    '''

    def __init__(self):
        super().__init__()
        self.__files = LaunchConfiguration(ARG_NAME, default='')
        # the node these overrides are for, set once it is created; None for a composable node
        self.node = None

    def describe(self) -> Text:
        return f'MergedOverrides({ARG_NAME})'

    def perform(self, context: LaunchContext) -> Text:
        paths = [os.path.abspath(os.path.expanduser(f))
                 for f in perform_substitutions(context, [self.__files]).split()]
        merged = launch_overrides(context, paths)
        # performed with the parameters, once the node's name is resolved
        full_name = None if self.node is None else full_node_name(self.node)
        content = node_sections(merged, full_name)
        if not any(section.get('ros__parameters') for section in content.values()):
            # nothing to override for this node: every such node shares one empty file
            empty = launch_global(context, 'simple_launch_empty_overrides', dict)
            if 'path' not in empty:
                empty['path'] = write_overrides(content, paths)
            return empty['path']
        path = write_overrides(content, paths)
        get_logger('simple_launch').debug(f'{ARG_NAME}: {full_name or "unnamed node"}: {path}')
        return path
